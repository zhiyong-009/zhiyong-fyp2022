#include <ros/ros.h>
#include <ros/package.h>

#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_utils/UnmodifiedTrajectory.h>

#include <gtest/gtest.h>

std::unique_ptr<ros::NodeHandle> nh;
ros::ServiceClient unmodified_trajectory_client;
ros::Publisher test_trajectory_pub;

TEST(TestSuite, testSrvExistence)
{
  unmodified_trajectory_client = nh->serviceClient<ram_utils::UnmodifiedTrajectory>(
      "ram/buffer/get_unmodified_trajectory");
  bool exists(unmodified_trajectory_client.waitForExistence(ros::Duration(5)));
  EXPECT_TRUE(exists);
}

TEST(TestSuite, testVerifyTrajectoryNotInBuffer)
{
  ros::Time timestamp = ros::Time::now();

  // Service request
  ram_utils::UnmodifiedTrajectory srv;
  srv.request.generated = timestamp;

  bool success(unmodified_trajectory_client.call(srv));
  EXPECT_TRUE(success);
  EXPECT_FALSE(srv.response.error.empty());
}

TEST(TestSuite, testVerifyTrajectoryInBuffer)
{
  ram_msgs::AdditiveManufacturingTrajectory trajectory;
  ros::Time timestamp = ros::Time::now();
  trajectory.generated = timestamp;
  trajectory.modified = timestamp;
  test_trajectory_pub.publish(trajectory);
  ros::Duration(2).sleep();

  // Service request
  ram_utils::UnmodifiedTrajectory srv;
  srv.request.generated = timestamp;

  bool success(unmodified_trajectory_client.call(srv));
  EXPECT_FALSE(!srv.response.error.empty());
  EXPECT_TRUE(success);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_utils_test_client");
  nh.reset(new ros::NodeHandle);
  test_trajectory_pub = nh->advertise<ram_msgs::AdditiveManufacturingTrajectory>("ram/trajectory", 5, true);

  // Make sure we have at least one subscriber to this topic
  while (test_trajectory_pub.getNumSubscribers() != 1)
  {
    ROS_WARN_STREAM_THROTTLE(1, test_trajectory_pub.getTopic() + " has zero subscriber, waiting...");
    ros::Duration(0.1).sleep();
  }

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
