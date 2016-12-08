#include <rodeobot/wander.h>
#include <angles/angles.h>
#include <gtest/gtest.h>

TEST(WanderTest, scan)
{
  Scan scan;

  // Note: we need to pick integers where the degrees <> radians round trip works.

  scan.init();
  for (int i = 0; i < 360; ++i)
    scan.putValue(angles::from_degrees(i), i == 190 ? 40 : 50);
  ASSERT_EQ(static_cast<int>(angles::to_degrees(scan.getBestAngle())), 190) << "Best angle should be 190";

  scan.init();
  for (int i = 0; i < 360; i += 5)
    scan.putValue(angles::from_degrees(i), i == 190 ? 40 : 50);
  ASSERT_EQ(angles::to_degrees(scan.getBestAngle()), 190) << "Best angle should be 190";
}

TEST(WanderTest, drive)
{
  DriveModel model;

  model.initDrive();
  ASSERT_EQ(model.computeLinearX(0.0, 0.0), 0.0);
  ASSERT_EQ(model.computeLinearX(0.0, 1.0), model.accel_);
  ASSERT_EQ(model.computeLinearX(0.0, 10.0), model.max_v_); // Assumes max_v_ / accel_ is <= 10
  ASSERT_EQ(model.computeLinearX(100.0, 0.0), model.max_v_);

  model.initStop();
  ASSERT_EQ(model.computeLinearX(0.0, 0.0), 0.0);
  ASSERT_EQ(model.computeLinearX(model.max_v_, 10.0), 0.0); // Assumes max_v_ / accel_ is <= 10
}

TEST(WanderTest, rotate)
{
  RotateModel model;

  model.initGoalAngle(-1.5, 1.5);
  ASSERT_EQ(model.clockwise(), true);
  ASSERT_EQ(model.computeAngularZ(-1.5, 0.0, 0.0), 0.0);
  ASSERT_EQ(model.computeAngularZ(-1.5, 0.0, 1.0), model.accel_);
  ASSERT_EQ(model.computeAngularZ(-1.5, 0.0, 10.0), model.max_v_); // Assumes max_v_ / accel_ is <= 10

  model.initGoalAngle(1.5, -1.5);
  ASSERT_EQ(model.clockwise(), false);
  ASSERT_EQ(model.computeAngularZ(1.5, 0.0, 0.0), 0.0);
  ASSERT_EQ(model.computeAngularZ(1.5, 0.0, 1.0), -model.accel_);
  ASSERT_EQ(model.computeAngularZ(1.5, 0.0, 10.0), -model.max_v_); // Assumes max_v_ / accel_ is <= 10
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
