#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include <unistd.h>

#include <cstdio>
#include <limits>
#include <stdexcept>

#include "relocalization_manager/relocalization_manager.hpp"
#include "relocalization_manager/refinement_policy.hpp"

namespace
{
using relocalization_manager::SmallGicpParams;
using relocalization_manager::SmallGicpVerifier;

TEST(VerifiedCorrection, BoundsTranslationAndYawWithoutChangingInitialAlignment)
{
  const auto from = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation() = Eigen::Vector3d(0.20, -0.10, 0.04);
  target.linear() = Eigen::AngleAxisd(0.08, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto step = relocalization_manager::blendVerifiedCorrection(from, target, 0.25);
  EXPECT_TRUE(step.translation().isApprox(Eigen::Vector3d(0.05, -0.025, 0.01), 1e-12));
  EXPECT_NEAR(Eigen::AngleAxisd(step.linear()).angle(), 0.02, 1e-12);
  EXPECT_TRUE(relocalization_manager::blendVerifiedCorrection(from, target, 1.0).isApprox(target));
  EXPECT_THROW(relocalization_manager::blendVerifiedCorrection(from, target, 0.0), std::invalid_argument);
  EXPECT_THROW(relocalization_manager::blendVerifiedCorrection(from, target, 1.01), std::invalid_argument);
}

class GicpVerification : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char path[] = "/tmp/pnx_gicp_verification_XXXXXX";
    const int fd = mkstemp(path);
    ASSERT_GE(fd, 0);
    close(fd);
    path_ = path;
    pcl::PointCloud<pcl::PointXYZ> target;
    for (int a = -12; a <= 12; ++a) {
      for (int b = -12; b <= 12; ++b) {
        target.push_back(pcl::PointXYZ(0.08F * a, 0.08F * b, 0.0F));
        target.push_back(pcl::PointXYZ(1.2F, 0.08F * a, 0.08F * b + 1.0F));
        target.push_back(pcl::PointXYZ(0.08F * a, 1.3F, 0.08F * b + 1.0F));
      }
    }
    ASSERT_EQ(pcl::io::savePCDFileBinary(path_, target), 0);
    truth_.translation() = Eigen::Vector3d(0.08, -0.06, 0.04);
    truth_.linear() = Eigen::AngleAxisd(0.04, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pcl::transformPointCloud(target, source_, truth_.inverse().matrix());
    params_.gravity_aligned = true;
    params_.translation_epsilon = 0.005;
    params_.rotation_epsilon = 0.003490658503988659;
    params_.max_iterations = 60;
    params_.global_leaf_size = 0.04F;
    params_.registered_leaf_size = 0.04F;
    params_.min_inliers = 200;
  }

  void TearDown() override
  {
    if (!path_.empty()) {
      std::remove(path_.c_str());
    }
  }

  std::string path_;
  pcl::PointCloud<pcl::PointXYZ> source_;
  Eigen::Isometry3d truth_{Eigen::Isometry3d::Identity()};
  SmallGicpParams params_;
};

TEST_F(GicpVerification, MillimeterToleranceRecoversKnownPose)
{
  SmallGicpVerifier verifier(rclcpp::get_logger("test"), params_, {});
  ASSERT_TRUE(verifier.loadGlobalMap(path_));
  const auto result = verifier.verify(source_, Eigen::Isometry3d::Identity());
  ASSERT_TRUE(result.accepted);
  EXPECT_LT((result.transform.translation() - truth_.translation()).norm(), 0.02);
  EXPECT_LT(
    Eigen::AngleAxisd(truth_.linear().transpose() * result.transform.linear()).angle(), 0.01);
}

TEST_F(GicpVerification, ConvergenceDoesNotBypassPoseDeltaGate)
{
  params_.max_delta_xy = 0.02;
  SmallGicpVerifier verifier(rclcpp::get_logger("test"), params_, {});
  ASSERT_TRUE(verifier.loadGlobalMap(path_));
  const auto result = verifier.verify(source_, Eigen::Isometry3d::Identity());
  EXPECT_TRUE(result.converged);
  EXPECT_FALSE(result.accepted);
}

TEST_F(GicpVerification, InvalidToleranceIsRejected)
{
  params_.translation_epsilon = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(SmallGicpVerifier(rclcpp::get_logger("test"), params_, {}), std::invalid_argument);
  params_.translation_epsilon = 0.005;
  params_.rotation_epsilon = 0.0;
  EXPECT_THROW(SmallGicpVerifier(rclcpp::get_logger("test"), params_, {}), std::invalid_argument);
}
}  // namespace
