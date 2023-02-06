#include <gtest/gtest.h>
#include <industrial_robot_angle_conversions/poses.hpp>
#include <ros/ros.h>

using namespace industrial_robot_angle_conversions;
using Eigen::Quaterniond;
using Eigen::Vector3d;

TEST(TestSuite, RPYtoQuaternion)
{
  Vector3d rpy;
  Quaterniond q;

  rpy << 20, 86, 32;
  rpy *= deg_2_rad;
  q = RPYtoQuaternion(rpy);
  EXPECT_NEAR(q.x(), 0.307207, 1e-6);
  EXPECT_NEAR(q.y(), 0.610614, 1e-6);
  EXPECT_NEAR(q.z(), 0.312366, 1e-6);
  EXPECT_NEAR(q.w(), 0.659699, 1e-6);

  rpy << 160, -25, 55;
  rpy *= deg_2_rad;
  q = RPYtoQuaternion(rpy);
  EXPECT_NEAR(q.x(), 0.8354744, 1e-6);
  EXPECT_NEAR(q.y(), -0.4772923, 1e-6);
  EXPECT_NEAR(q.z(), -0.1107864, 1e-6);
  EXPECT_NEAR(q.w(), 0.2487991, 1e-6);
}

TEST(TestSuite, quaternionToRPY)
{
  Quaterniond q;
  Vector3d rpy;

  q.x() = 0.307207;
  q.y() = 0.610614;
  q.z() = 0.312366;
  q.w() = 0.659699;
  rpy = quaternionToRPY(q);
  EXPECT_NEAR(rpy[0], 20 * deg_2_rad, 1e-5);
  EXPECT_NEAR(rpy[1], 86 * deg_2_rad, 1e-5);
  EXPECT_NEAR(rpy[2], 32 * deg_2_rad, 1e-5);

  q.x() = 0.8354744;
  q.y() = -0.4772923;
  q.z() = -0.1107864;
  q.w() = 0.2487991;
  rpy = quaternionToRPY(q);
  EXPECT_NEAR(rpy[0], 160 * deg_2_rad, 1e-5);
  EXPECT_NEAR(rpy[1], -25 * deg_2_rad, 1e-5);
  EXPECT_NEAR(rpy[2], 55 * deg_2_rad, 1e-5);
}

TEST(TestSuite, FanucWPRtoQuaternion)
{
  Quaterniond q;
  Vector3d wpr;

  wpr << -50, 125, 67;
  wpr *= deg_2_rad;
  q = fanucWPRtoQuaternion(wpr);
  EXPECT_NEAR(q.x(), -0.606432, 1e-6);
  EXPECT_NEAR(q.y(), 0.562658, 1e-6);
  EXPECT_NEAR(q.z(), 0.543574, 1e-6);
  EXPECT_NEAR(q.w(), 0.142067, 1e-6);

  wpr << 160, 55, -35;
  wpr *= deg_2_rad;
  q = fanucWPRtoQuaternion(wpr);
  EXPECT_NEAR(q.x(), 0.857216, 1e-6);
  EXPECT_NEAR(q.y(), -0.186206, 1e-6);
  EXPECT_NEAR(q.z(), -0.480004, 1e-6);
  EXPECT_NEAR(q.w(), 0.0101579, 1e-6);
}

TEST(TestSuite, quaternionToFanucWPR)
{
  Quaterniond q;
  Vector3d wpr;

  q.x() = -0.606432;
  q.y() = 0.562658;
  q.z() = 0.543574;
  q.w() = 0.142067;
  wpr = quaternionToFanucWPR(q);
  EXPECT_NEAR(wpr[0], -50 * deg_2_rad, 1e-5);
  EXPECT_NEAR(wpr[1], 125 * deg_2_rad, 1e-5);
  EXPECT_NEAR(wpr[2], 67 * deg_2_rad, 1e-5);

  q.x() = 0.857216;
  q.y() = -0.186206;
  q.z() = -0.480004;
  q.w() =  0.0101579;
  wpr = quaternionToFanucWPR(q);
  // The solution found is NOT 160, 55, -35 but a different combination
  // of angles that represent the same orientation.
  // See second test of FanucWPRtoQuaternion
  EXPECT_NEAR(wpr[0], -20 * deg_2_rad, 1e-5);
  EXPECT_NEAR(wpr[1], 125 * deg_2_rad, 1e-5);
  EXPECT_NEAR(wpr[2], 145 * deg_2_rad, 1e-5);
}

TEST(TestSuite, isometryToFanucPose)
{
  Eigen::Isometry3d p(Eigen::Isometry3d::Identity());
  Eigen::Vector3d xyz_mm, wpr_deg;

  p.translation().x() = 0.5;
  p.translation().y() = -1.0;
  p.translation().z() = 0.2;
  p.linear() = (Eigen::AngleAxisd(0.35, Vector3d::UnitX())
                *  Eigen::AngleAxisd(-0.1, Vector3d::UnitY())
                *  Eigen::AngleAxisd(0.6, Vector3d::UnitZ())).matrix();

  isometryToFanucPose(p, xyz_mm, wpr_deg);
  EXPECT_NEAR(xyz_mm.x(), 500, 1e-8);
  EXPECT_NEAR(xyz_mm.y(), -1000, 1e-8);
  EXPECT_NEAR(xyz_mm.z(), 200, 1e-8);
  EXPECT_NEAR(wpr_deg[0], 13.8274, 1e-4);
  EXPECT_NEAR(wpr_deg[1], -15.7247, 1e-4);
  EXPECT_NEAR(wpr_deg[2], 31.445, 1e-4);

  p.translation().x() = -1.0;
  p.translation().y() = -0.8;
  p.translation().z() = -0.1;
  p.linear() = (Eigen::AngleAxisd(-0.45, Vector3d::UnitX())
                *  Eigen::AngleAxisd(1.65, Vector3d::UnitY())
                *  Eigen::AngleAxisd(0.9, Vector3d::UnitZ())).matrix();

  isometryToFanucPose(p, xyz_mm, wpr_deg);
  EXPECT_NEAR(xyz_mm.x(), -1000, 1e-8);
  EXPECT_NEAR(xyz_mm.y(), -800, 1e-8);
  EXPECT_NEAR(xyz_mm.z(), -100, 1e-8);
  EXPECT_NEAR(wpr_deg[0], 99.3487, 1e-4);
  EXPECT_NEAR(wpr_deg[1], 63.9867, 1e-4);
  EXPECT_NEAR(wpr_deg[2], 96.4387, 1e-4);
}

TEST(TestSuite, fanucPoseToIsometry)
{
  Eigen::Vector3d xyz_mm, wpr_deg;
  Eigen::Isometry3d p;
  Eigen::Quaterniond q;

  xyz_mm << 500, -1000, 200;
  wpr_deg << 13.8274, -15.7247, 31.445;
  fanucPoseToIsometry(xyz_mm, wpr_deg, p);
  q = p.linear();
  EXPECT_NEAR(p.translation().x(), 0.5, 1e-8);
  EXPECT_NEAR(p.translation().y(), -1, 1e-8);
  EXPECT_NEAR(p.translation().z(), 0.2, 1e-8);
  EXPECT_NEAR(q.x(), 0.15158, 1e-5);
  EXPECT_NEAR(q.y(), -0.0984059, 1e-5);
  EXPECT_NEAR(q.z(), 0.28233, 1e-5);
  EXPECT_NEAR(q.w(), 0.942141, 1e-5);

  xyz_mm << -1000, -800, -100;
  wpr_deg << 99.3487, 63.9867, 96.4387;
  fanucPoseToIsometry(xyz_mm, wpr_deg, p);
  q = p.linear();
  EXPECT_NEAR(p.translation().x(), -1, 1e-8);
  EXPECT_NEAR(p.translation().y(), -0.8, 1e-8);
  EXPECT_NEAR(p.translation().z(), -0.1, 1e-8);
  EXPECT_NEAR(q.x(), 0.17513, 1e-5);
  EXPECT_NEAR(q.y(), 0.710599, 1e-5);
  EXPECT_NEAR(q.z(), 0.140142, 1e-5);
  EXPECT_NEAR(q.w(), 0.666887, 1e-5);
}

int main(int argc,
         char **argv)
{
  ros::init(argc, argv, "angles_and_poses");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
