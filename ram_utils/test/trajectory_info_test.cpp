#include <ros/ros.h>
#include <ros/package.h>

#include <unique_id/unique_id.h>

#include <ram_msgs/AdditiveManufacturingTrajectory.h>

#include <ram_utils/GetTrajectorySize.h>
#include <ram_utils/GetNumberOfLayersLevels.h>
#include <ram_utils/GetLayerSize.h>

#include <gtest/gtest.h>

std::unique_ptr<ros::NodeHandle> nh;
ros::Publisher test_trajectory_pub;
ram_msgs::AdditiveManufacturingTrajectory trajectory;

// Pose selector services
ros::ServiceClient get_trajectory_size_client;
ros::ServiceClient get_number_of_layers_levels_client;
ros::ServiceClient get_layer_size_client;

TEST(TestSuite, testSrvsExistence)
{
  get_trajectory_size_client = nh->serviceClient<ram_utils::GetTrajectorySize>("ram/information/get_trajectory_size");
  bool exists_1(get_trajectory_size_client.waitForExistence(ros::Duration(5)));
  EXPECT_TRUE(exists_1);

  get_number_of_layers_levels_client = nh->serviceClient<ram_utils::GetNumberOfLayersLevels>(
      "ram/information/get_number_of_layers_levels");
  bool exists_2(get_number_of_layers_levels_client.waitForExistence(ros::Duration(5)));
  EXPECT_TRUE(exists_2);

  get_layer_size_client = nh->serviceClient<ram_utils::GetLayerSize>("ram/information/get_layer_size");
  bool exists_3(get_layer_size_client.waitForExistence(ros::Duration(5)));
  EXPECT_TRUE(exists_3);
}

TEST(TestSuite, selectPosesInEmptyTrajectory)
{
  // Publish empty trajectory
  trajectory.poses.clear();
  ros::Time timestamp = ros::Time::now();
  trajectory.generated = timestamp;
  trajectory.modified = timestamp;
  test_trajectory_pub.publish(trajectory);
  ros::Duration(2).sleep();

  ram_utils::GetTrajectorySize get_trajectory_size_srv;
  get_trajectory_size_client.call(get_trajectory_size_srv);
  bool success_1 = (get_trajectory_size_srv.response.trajectory_size != 0);
  EXPECT_FALSE(success_1);

  ram_utils::GetNumberOfLayersLevels get_number_of_layers_srv;
  bool success_2(get_number_of_layers_levels_client.call(get_number_of_layers_srv));
  EXPECT_TRUE(success_2);

  ram_utils::GetLayerSize get_layer_size_srv;
  bool success_3(get_layer_size_client.call(get_layer_size_srv));
  EXPECT_TRUE(success_3);
}

TEST(TestSuite, getTrajectorySize)
{
  trajectory.poses.clear();
  // 3 layers. 1 pose in the first layer, 2 poses in the second, etc.
  for (int i(0); i < 3; ++i)
  {
    for (int j(0); j <= i; ++j)
    {
      ram_msgs::AdditiveManufacturingPose pose;
      pose.layer_level = i;
      pose.unique_id = unique_id::toMsg(unique_id::fromRandom());
      trajectory.poses.push_back(pose);
    }
  }
  ros::Time timestamp = ros::Time::now();
  trajectory.generated = timestamp;
  trajectory.modified = timestamp;
  test_trajectory_pub.publish(trajectory);
  ros::Duration(2).sleep();

  ram_utils::GetTrajectorySize srv;
  bool success(get_trajectory_size_client.call(srv));
  bool response = (srv.response.trajectory_size == 6);
  EXPECT_TRUE(success);
  EXPECT_TRUE(response);
}

TEST(TestSuite, numberOfLayersLevels)
{
  ram_utils::GetNumberOfLayersLevels srv;
  get_number_of_layers_levels_client.call(srv);
  bool success(get_number_of_layers_levels_client.call(srv));
  bool response = (srv.response.number_of_layers == 3);

  EXPECT_TRUE(success);
  EXPECT_TRUE(response);
}

TEST(TestSuite, layerSize)
{
  ram_utils::GetLayerSize srv;
  srv.request.layer_level = 2;
  get_layer_size_client.call(srv);
  bool success(get_layer_size_client.call(srv));
  bool response = (srv.response.layer_size == 3);

  EXPECT_TRUE(success);
  EXPECT_TRUE(response);
}

int main(int argc,
         char **argv)
{
  ros::init(argc, argv, "ram_utils_trajectory_info_test");
  nh.reset(new ros::NodeHandle);

  test_trajectory_pub = nh->advertise<ram_msgs::AdditiveManufacturingTrajectory>("ram/trajectory", 10, true);

  // Make sure we have at least one subscriber to this topic
  while (test_trajectory_pub.getNumSubscribers() != 1)
  {
    ROS_WARN_STREAM_THROTTLE(1, test_trajectory_pub.getTopic() + " has zero subscriber, waiting...");
    ros::Duration(0.1).sleep();
  }

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
