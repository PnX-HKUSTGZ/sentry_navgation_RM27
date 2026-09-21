#include "minco_core/components/cached_trajectory_policy.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace minco_planner::cached_trajectory_policy {

TEST(CachedTrajectoryPolicy, BoundsCollisionRelatedReuse) {
  EXPECT_TRUE(reuseDurationAllowed("FOOTPRINT_COLLISION", 0.75, 0.75));
  EXPECT_FALSE(reuseDurationAllowed("FOOTPRINT_COLLISION", 0.751, 0.75));
  EXPECT_FALSE(reuseDurationAllowed("LOCAL_SEED_COLLISION", 2.0, 0.75));
}

TEST(CachedTrajectoryPolicy, BoundsNumericalFailureReuse) {
  EXPECT_TRUE(reuseDurationAllowed("OPTIMIZER_FAILED", 0.75, 0.75));
  EXPECT_FALSE(reuseDurationAllowed("OPTIMIZER_FAILED", 0.751, 0.75));
  EXPECT_FALSE(reuseDurationAllowed(
      "OPTIMIZER_FAILED", std::numeric_limits<double>::quiet_NaN(), 0.75));
}

} // namespace minco_planner::cached_trajectory_policy
