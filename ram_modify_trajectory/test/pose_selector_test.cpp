#include <ros/ros.h>
#include <ros/package.h>

#include <unique_id/unique_id.h>

#include <ram_msgs/AdditiveManufacturingTrajectory.h>

#include <ram_modify_trajectory/GetPosesFromTrajectory.h>
#include <ram_modify_trajectory/GetPosesFromLayersList.h>
#include <ram_modify_trajectory/GetPosesFromLayer.h>

#include <gtest/gtest.h>

std::unique_ptr<ros::NodeHandle> nh;
ros::Publisher test_trajectory_pub;
ram_msgs::AdditiveManufacturingTrajectory trajectory;

// Pose selector services
ros::ServiceClient get_poses_from_trajectory_client;
ros::ServiceClient get_poses_from_layers_list_client;
ros::ServiceClient get_poses_from_layer_client;

TEST(TestSuite, testSrvsExistence)
{
  get_poses_from_trajectory_client = nh->serviceClient<ram_modify_trajectory::GetPosesFromTrajectory>(
      "ram/pose_selector/get_poses_from_trajectory");
  bool exists_4(get_poses_from_trajectory_client.waitForExistence(ros::Duration(5)));
  EXPECT_TRUE(exists_4);

  get_poses_from_layers_list_client = nh->serviceClient<ram_modify_trajectory::GetPosesFromLayersList>(
      "ram/pose_selector/get_poses_from_layers_list");
  bool exists_5(get_poses_from_layers_list_client.waitForExistence(ros::Duration(5)));
  EXPECT_TRUE(exists_5);

  get_poses_from_layer_client = nh->serviceClient<ram_modify_trajectory::GetPosesFromLayer>(
      "ram/pose_selector/get_poses_from_layer");
  bool exists_6(get_poses_from_layer_client.waitForExistence(ros::Duration(5)));
  EXPECT_TRUE(exists_6);
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

  ram_modify_trajectory::GetPosesFromTrajectory srv_1;
  bool success_4(get_poses_from_trajectory_client.call(srv_1));
  bool success_5(srv_1.response.error.empty());
  EXPECT_TRUE(success_4);
  EXPECT_FALSE(success_5);

  ram_modify_trajectory::GetPosesFromLayersList srv_2;
  bool success_6(get_poses_from_layers_list_client.call(srv_2));
  bool success_7(srv_2.response.error.empty());
  EXPECT_TRUE(success_6);
  EXPECT_FALSE(success_7);

  ram_modify_trajectory::GetPosesFromLayer srv_3;
  bool success_8(get_poses_from_layer_client.call(srv_3));
  bool success_9(srv_3.response.error.empty());
  EXPECT_TRUE(success_8);
  EXPECT_FALSE(success_9);

}

TEST(TestSuite, getPosesFromTrajectory)
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

  ram_modify_trajectory::GetPosesFromTrajectory srv;
  srv.request.pose_index_list = {0, 1, 2, 3};
  bool success(get_poses_from_trajectory_client.call(srv));

  // Verify uuid
  bool response = true;
  for (unsigned i(0); i < srv.request.pose_index_list.size(); ++i)
  {
    if (trajectory.poses[i].unique_id.uuid != srv.response.poses[i].unique_id.uuid) // uuid are not equals
    {
      response = false;
      break;
    }
  }
  bool success_2(srv.response.error.empty());
  EXPECT_TRUE(success);
  EXPECT_TRUE(response);
  EXPECT_TRUE(success_2);
}

TEST(TestSuite, getPosesFromLayersList)
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

  ram_modify_trajectory::GetPosesFromLayersList srv;
  srv.request.layer_level_list = {2};
  bool success(get_poses_from_layers_list_client.call(srv));
  // Verify uuid
  bool response = true;
  for (unsigned i(0); i < 3; ++i) // 3 poses in this layer
  {
    if (trajectory.poses[i + 3].unique_id.uuid != srv.response.poses[i].unique_id.uuid) // uuid are not equals
    {
      response = false;
      break;
    }
  }
  bool success_2(srv.response.error.empty());
  EXPECT_TRUE(success);
  EXPECT_TRUE(response);
  EXPECT_TRUE(success_2);
}

TEST(TestSuite, getPosesFromLayer)
{
  ram_modify_trajectory::GetPosesFromLayer srv;
  srv.request.layer_level = 2;
  srv.request.index_list_relative = {2}; // last pose in the trajectory

  bool success(get_poses_from_layer_client.call(srv));
  // Verify uuid
  bool response = (trajectory.poses.back().unique_id.uuid == srv.response.poses[0].unique_id.uuid);
  bool success_2(srv.response.error.empty());
  EXPECT_TRUE(success);
  EXPECT_TRUE(response);
  EXPECT_TRUE(success_2);
}

TEST(TestSuite, trajectoryWithEmptyLayer)
{
  // 4 layers. 4 poses par layer. third layer is empty
  trajectory.poses.clear();
  for (int i(0); i < 4; ++i)
  {
    if (i == 2)
      continue;
    for (int j(0); j < 4; ++j)
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

  ram_modify_trajectory::GetPosesFromLayersList srv;
  srv.request.layer_level_list = {2};
  bool success(get_poses_from_layers_list_client.call(srv));
  bool response(srv.response.poses.empty());
  EXPECT_TRUE(success);
  EXPECT_TRUE(response);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "modify_trajectory_test");
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
