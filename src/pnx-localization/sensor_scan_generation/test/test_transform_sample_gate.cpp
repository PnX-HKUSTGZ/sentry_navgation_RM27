#include <gtest/gtest.h>

#include "sensor_scan_generation/transform_sample_gate.hpp"

namespace sensor_scan_generation
{
namespace
{

TEST(TransformSampleGate, MissingAllTransformsRequestsZeroPublication)
{
  TransformSampleGate gate;

  const TransformSampleDecision decision = gate.evaluate(false, false);

  EXPECT_FALSE(decision.publish_sample);
  EXPECT_TRUE(decision.reset_twist_history);
  EXPECT_FALSE(decision.recovered);
}

TEST(TransformSampleGate, EitherRequiredTransformFailureDropsTheWholeSample)
{
  TransformSampleGate missing_robot_base_gate;
  TransformSampleGate missing_chassis_gate;

  const TransformSampleDecision missing_robot_base = missing_robot_base_gate.evaluate(false, true);
  const TransformSampleDecision missing_chassis = missing_chassis_gate.evaluate(true, false);

  EXPECT_FALSE(missing_robot_base.publish_sample);
  EXPECT_TRUE(missing_robot_base.reset_twist_history);
  EXPECT_FALSE(missing_chassis.publish_sample);
  EXPECT_TRUE(missing_chassis.reset_twist_history);
}

TEST(TransformSampleGate, RecoveryPublishesOnlyAfterRequestingAColdStart)
{
  TransformSampleGate gate;

  const TransformSampleDecision first_valid = gate.evaluate(true, true);
  EXPECT_TRUE(first_valid.publish_sample);
  EXPECT_TRUE(first_valid.reset_twist_history);
  EXPECT_FALSE(first_valid.recovered);

  const TransformSampleDecision tracking = gate.evaluate(true, true);
  EXPECT_TRUE(tracking.publish_sample);
  EXPECT_FALSE(tracking.reset_twist_history);

  const TransformSampleDecision failed = gate.evaluate(true, false);
  EXPECT_FALSE(failed.publish_sample);
  EXPECT_TRUE(failed.reset_twist_history);

  const TransformSampleDecision recovered = gate.evaluate(true, true);
  EXPECT_TRUE(recovered.publish_sample);
  EXPECT_TRUE(recovered.reset_twist_history);
  EXPECT_TRUE(recovered.recovered);

  const TransformSampleDecision tracking_again = gate.evaluate(true, true);
  EXPECT_TRUE(tracking_again.publish_sample);
  EXPECT_FALSE(tracking_again.reset_twist_history);
  EXPECT_FALSE(tracking_again.recovered);
}

}  // namespace
}  // namespace sensor_scan_generation
