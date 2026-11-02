#include <gtest/gtest.h>

#include <array>
#include <limits>

#include "common_lib.h"

namespace
{
using Filter = esekfom::esekf<state_output, 30, input_ikfom>;
using Vector6 = Eigen::Matrix<double, 6, 1>;
Vector6 measurement;
Vector6 variance;
std::array<bool, 6> saturated;
bool valid_measurement;
bool corrupt_candidate;

Eigen::Matrix<double, 30, 1> process(state_output &, const input_ikfom &)
{
  return Eigen::Matrix<double, 30, 1>::Zero();
}
Eigen::Matrix<double, 30, 30> processJacobian(state_output &, const input_ikfom &)
{
  return Eigen::Matrix<double, 30, 30>::Zero();
}
void lidar(state_output &, Eigen::Matrix3d, Eigen::Matrix3d,
           esekfom::dyn_share_modified<double> &) {}
void imu(state_output & state, esekfom::dyn_share_modified<double> & observation)
{
  observation.valid = valid_measurement;
  observation.z_IMU.head<3>() = measurement.head<3>() - state.omg - state.bg;
  observation.z_IMU.tail<3>() = measurement.tail<3>() - state.acc - state.ba;
  observation.R_IMU = variance;
  for (int axis = 0; axis < 6; ++axis) {
    observation.satu_check[axis] = saturated[axis];
  }
  if (corrupt_candidate) {
    state.pos.x() = std::numeric_limits<double>::quiet_NaN();
  }
}

class ImuUpdate : public ::testing::Test
{
protected:
  void SetUp() override
  {
    measurement << 0.1, -0.2, 0.3, 0.4, -0.5, 0.6;
    variance.setConstant(0.25);
    saturated.fill(false);
    valid_measurement = true;
    corrupt_candidate = false;
    filter.init_dyn_share_modified_3h(process, processJacobian, lidar, imu);
  }
  void expectUnchanged(const state_output & state, const Filter::cov & covariance)
  {
    Filter::vectorized_state delta;
    filter.x_.boxminus(delta, state);
    EXPECT_TRUE(delta.isZero(0));
    EXPECT_TRUE((filter.P_.array() == covariance.array()).all());
  }
  Filter filter;
};

TEST_F(ImuUpdate, MatchesAnalyticLinearPosteriorIncludingCrossCovariance)
{
  filter.P_(0, 15) = filter.P_(15, 0) = 0.2;
  const Filter::cov prior = filter.P_;
  Eigen::Matrix<double, 6, 30> H = Eigen::Matrix<double, 6, 30>::Zero();
  for (int i = 0; i < 6; ++i) {H(i, 15 + i) = H(i, 24 + i) = 1;}
  // With identity prior in the observed blocks, innovation variance is 2 + 0.25.
  const Eigen::Matrix<double, 30, 6> K = prior * H.transpose() / 2.25;
  const Filter::vectorized_state expected_delta = K * measurement;
  const Filter::cov expected_covariance = prior - K * H * prior;
  const state_output before = filter.x_;
  ASSERT_TRUE(filter.update_iterated_dyn_share_IMU());
  Filter::vectorized_state actual_delta;
  filter.x_.boxminus(actual_delta, before);
  EXPECT_TRUE(actual_delta.isApprox(expected_delta, 1e-12));
  EXPECT_TRUE(filter.P_.isApprox(expected_covariance, 1e-12));
  EXPECT_STREQ(filter.last_imu_update_reason(), "accepted");
}

TEST_F(ImuUpdate, RejectsNonfiniteInnovationWithoutChangingPrior)
{
  const auto state = filter.x_;
  const auto covariance = filter.P_;
  for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity()}) {
    measurement(0) = bad;
    EXPECT_FALSE(filter.update_iterated_dyn_share_IMU());
    EXPECT_STREQ(filter.last_imu_update_reason(), "nonfinite_innovation");
    expectUnchanged(state, covariance);
  }
}

TEST_F(ImuUpdate, RejectsInvalidNoiseWithoutChangingPrior)
{
  const auto state = filter.x_;
  const auto covariance = filter.P_;
  for (double bad : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::quiet_NaN()}) {
    variance(4) = bad;
    EXPECT_FALSE(filter.update_iterated_dyn_share_IMU());
    EXPECT_STREQ(filter.last_imu_update_reason(), "invalid_measurement_noise");
    expectUnchanged(state, covariance);
  }
}

TEST_F(ImuUpdate, RejectsInvalidModelAndPosteriorAtomically)
{
  const auto state = filter.x_;
  const auto covariance = filter.P_;
  valid_measurement = false;
  EXPECT_FALSE(filter.update_iterated_dyn_share_IMU());
  EXPECT_STREQ(filter.last_imu_update_reason(), "invalid_measurement");
  expectUnchanged(state, covariance);
  valid_measurement = true;
  corrupt_candidate = true;
  EXPECT_FALSE(filter.update_iterated_dyn_share_IMU());
  EXPECT_STREQ(filter.last_imu_update_reason(), "invalid_posterior_state");
  expectUnchanged(state, covariance);
}

TEST_F(ImuUpdate, RejectsFiniteInputOverflowWithoutChangingPrior)
{
  filter.x_.pos.x() = std::numeric_limits<double>::max();
  filter.P_(0, 15) = filter.P_(15, 0) = 0.8;
  measurement(0) = std::numeric_limits<double>::max();
  const auto state = filter.x_;
  const auto covariance = filter.P_;
  EXPECT_FALSE(filter.update_iterated_dyn_share_IMU());
  EXPECT_STREQ(filter.last_imu_update_reason(), "invalid_posterior_state");
  expectUnchanged(state, covariance);
}

TEST_F(ImuUpdate, RejectsInnovationCovarianceOverflowWithoutChangingPrior)
{
  filter.P_(15, 15) = std::numeric_limits<double>::max();
  filter.P_(24, 24) = std::numeric_limits<double>::max();
  const auto state = filter.x_;
  const auto covariance = filter.P_;
  EXPECT_FALSE(filter.update_iterated_dyn_share_IMU());
  EXPECT_STREQ(filter.last_imu_update_reason(), "invalid_innovation_covariance");
  expectUnchanged(state, covariance);
}

TEST_F(ImuUpdate, RejectsIndefiniteAsymmetricAndNonfinitePriorCovariance)
{
  const auto state = filter.x_;
  for (int kind = 0; kind < 3; ++kind) {
    filter.P_.setIdentity();
    if (kind == 0) {filter.P_(0, 0) = -0.01;}
    if (kind == 1) {filter.P_(0, 1) = 0.1;}
    if (kind == 2) {filter.P_(0, 0) = std::numeric_limits<double>::infinity();}
    const auto covariance = filter.P_;
    EXPECT_FALSE(filter.update_iterated_dyn_share_IMU());
    EXPECT_STREQ(filter.last_imu_update_reason(), "invalid_prior_covariance");
    expectUnchanged(state, covariance);
  }
}

TEST_F(ImuUpdate, RejectsInvalidPriorState)
{
  filter.x_.vel.x() = std::numeric_limits<double>::infinity();
  const auto covariance = filter.P_;
  EXPECT_FALSE(filter.update_iterated_dyn_share_IMU());
  EXPECT_STREQ(filter.last_imu_update_reason(), "invalid_prior_state");
  EXPECT_TRUE(std::isinf(filter.x_.vel.x()));
  EXPECT_TRUE(filter.P_.isApprox(covariance, 0));
  filter.x_.vel.setZero();
  filter.x_.rot = -Eigen::Matrix3d::Identity();
  EXPECT_FALSE(filter.update_iterated_dyn_share_IMU());
  EXPECT_TRUE(filter.x_.rot.isApprox(-Eigen::Matrix3d::Identity(), 0));
}

TEST_F(ImuUpdate, SaturatedAxesAreExcludedFromInnovationAndNoise)
{
  saturated[0] = true;
  measurement(0) = std::numeric_limits<double>::quiet_NaN();
  variance(0) = -1.0;
  ASSERT_TRUE(filter.update_iterated_dyn_share_IMU());
  EXPECT_DOUBLE_EQ(filter.x_.omg.x(), 0);
  EXPECT_DOUBLE_EQ(filter.x_.bg.x(), 0);
  EXPECT_DOUBLE_EQ(filter.P_(15, 15), 1);
  EXPECT_NEAR(filter.x_.omg.y(), measurement(1) / 2.25, 1e-12);
}

TEST_F(ImuUpdate, AllSaturatedAxesAreAnExactNoOp)
{
  saturated.fill(true);
  measurement.setConstant(std::numeric_limits<double>::quiet_NaN());
  variance.setZero();
  const auto state = filter.x_;
  const auto covariance = filter.P_;
  ASSERT_TRUE(filter.update_iterated_dyn_share_IMU());
  EXPECT_STREQ(filter.last_imu_update_reason(), "all_axes_saturated");
  expectUnchanged(state, covariance);
}

TEST_F(ImuUpdate, AcceptsZeroVarianceDirections)
{
  filter.P_.row(0).setZero();
  filter.P_.col(0).setZero();
  ASSERT_TRUE(filter.update_iterated_dyn_share_IMU());
  EXPECT_DOUBLE_EQ(filter.P_(0, 0), 0);
}

TEST_F(ImuUpdate, RepeatedUpdatesRemainPositiveDefinite)
{
  variance << 1e-10, 1e-7, 1e-4, 0.1, 10, 1e3;
  for (int i = 0; i < 300; ++i) {
    ASSERT_TRUE(filter.update_iterated_dyn_share_IMU()) << filter.last_imu_update_reason();
    ASSERT_TRUE(filter.P_.allFinite());
    ASSERT_TRUE(filter.P_.isApprox(filter.P_.transpose(), 1e-12));
    ASSERT_EQ(Eigen::LLT<Filter::cov>(filter.P_).info(), Eigen::Success);
  }
}

TEST_F(ImuUpdate, IterationSettingDoesNotCountTheSameImuSampleTwice)
{
  filter.set_maximum_iterations(5);
  ASSERT_TRUE(filter.update_iterated_dyn_share_IMU());
  EXPECT_NEAR(filter.x_.omg.x(), measurement(0) / 2.25, 1e-12);
  EXPECT_NEAR(filter.P_(15, 15), 1.0 - 1.0 / 2.25, 1e-12);
}
}  // namespace
