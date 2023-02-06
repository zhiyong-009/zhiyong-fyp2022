#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_listener.h>

#include <ram_msgs/AdditiveManufacturingPose.h>
#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_display/DisplayTrajectory.h>
#include <ram_display/DeleteTrajectory.h>

#include <gtest/gtest.h>

std::unique_ptr<ros::NodeHandle> nh;
bool use_gui;
ros::ServiceClient display_traj_client;
ros::ServiceClient delete_markers_client;

// DisplayTrajectory service
TEST(TestSuite, testDisplayServiceExistence)
{
  display_traj_client = nh->serviceClient<ram_display::DisplayTrajectory>("ram/display/add_trajectory");
  bool exists(display_traj_client.waitForExistence(ros::Duration(5)));
  EXPECT_TRUE(exists);
}

// Display: Pose number, wire + axis mode, one color per speed
TEST(TestSuite, testSendFirstTrajectoryFirstParameters)
{
  ros::Publisher pub = nh->advertise<ram_msgs::AdditiveManufacturingTrajectory>("ram/trajectory", 1, true);
  ram_msgs::AdditiveManufacturingTrajectory msg;
  Eigen::Isometry3d pose_isometry = Eigen::Isometry3d::Identity();
  for (unsigned j(1); j <= 2; j++) // 2 layers
  {
    for (unsigned i(0); i < 14; ++i)
    {
      pose_isometry.translation()[0] = cos(i * M_PI / 8);
      pose_isometry.translation()[1] = sin(i * M_PI / 8);
      pose_isometry.translation()[2] = j;

      geometry_msgs::Pose pose_srv;
      tf::poseEigenToMsg(pose_isometry, pose_srv);

      ram_msgs::AdditiveManufacturingPose ram_pose;
      ram_pose.pose = pose_srv;
      ram_pose.layer_level = j;
      ram_pose.params.speed = i;
      msg.poses.push_back(ram_pose);
    }
  }
  while (pub.getNumSubscribers() == 0)
  {
    ROS_WARN_STREAM("testSendFirstTrajectoryFirstParameters: No subscriber on topic ram/trajectory, waiting...");
    ros::Duration(0.5).sleep();
  }
  pub.publish(msg);
  ros::Duration(2).sleep();

  ram_display::DisplayTrajectory srv;
  srv.request.line_size = 0.01;
  srv.request.axis_size = 0.2;
  srv.request.display_type = 1;
  srv.request.display_axis = true;
  srv.request.color_mode = 1;
  srv.request.display_labels = false;
  bool success(display_traj_client.call(srv));
  EXPECT_TRUE(success);

  if (!srv.response.error.empty())
    ROS_ERROR_STREAM("testSendFirstTrajectoryFirstParameters: " << srv.response.error);

  EXPECT_TRUE(srv.response.error.empty());

  if (use_gui)
  {
    ROS_WARN_STREAM("testSendFirstTrajectoryFirstParameters: Press enter in the terminal to skip to the next test");
    while (std::cin.get() != '\n')
      ros::Duration(0.1).sleep();
  }
}

// Display: layer number, cylinder + axis mode, one color per layer
TEST(TestSuite, testSendSecondParameters)
{
  // We re-us the same trajectory, no need to publish a new one
  ram_display::DisplayTrajectory srv;
  srv.request.line_size = 0.01;
  srv.request.axis_size = 0.2;
  srv.request.display_type = 0;
  srv.request.color_mode = 0;
  srv.request.display_labels = true;
  srv.request.label_type = 0;
  srv.request.label_size = 0.09;
  bool success(display_traj_client.call(srv));
  EXPECT_TRUE(success);

  if (!srv.response.error.empty())
    ROS_ERROR_STREAM("testSendSecondParameters: " << srv.response.error);

  EXPECT_TRUE(srv.response.error.empty());

  if (use_gui)
  {
    ROS_WARN_STREAM("testSendSecondParameters: Press enter in the terminal to skip to the next test");
    while (std::cin.get() != '\n')
      ros::Duration(0.1).sleep();
  }
}

// Send and wrong value in display_type
TEST(TestSuite, testSendSecondWrongParameters)
{
  if (use_gui)
    return;

  ram_display::DisplayTrajectory srv;
  srv.request.display_type = 9;
  bool success(display_traj_client.call(srv));
  EXPECT_TRUE(success);

  if (!srv.response.error.empty())
    ROS_INFO_STREAM("testSendSecondWrongParameters" << srv.response.error);

  EXPECT_FALSE(srv.response.error.empty());
}

// DeleteTrajectory service
TEST(TestSuite, testDeleteMarkersServiceExistence)
{
  delete_markers_client = nh->serviceClient<ram_display::DeleteTrajectory>("ram/display/delete_trajectory");
  bool exists(delete_markers_client.waitForExistence(ros::Duration(5)));
  EXPECT_TRUE(exists);
}

// Call DeleteTrajectory
TEST(TestSuite, testDeleteMarkers)
{
  ram_display::DeleteTrajectory srv;
  bool success(delete_markers_client.call(srv));
  EXPECT_TRUE(success);

  if (use_gui)
  {
    ROS_WARN_STREAM("testDeleteMarkers: Press enter in the terminal to skip to the next test");
    while (std::cin.get() != '\n')
      ros::Duration(0.1).sleep();
  }

  if (use_gui)
    ROS_ERROR_STREAM("PLEASE CLOSE THE RViz WINDOW TO TERMINATE");
}

int main(int argc,
         char **argv)
{
  ros::init(argc, argv, "services_clients");
  nh.reset(new ros::NodeHandle);

  // Determine if we are running with RViz or not
  nh->param<bool>("use_gui", use_gui, false);
  std::string trajectory_frame;
  nh->param<std::string>("ram/trajectory_frame", trajectory_frame, "base");

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener listener(tf_buffer);

  bool fetched(false);
  while (!fetched && ros::ok())
  {
    try
    {
      tf_buffer.lookupTransform(trajectory_frame, "base", ros::Time(0), ros::Duration(5));
      fetched = true;
    }
    catch (...)
    {
      ROS_WARN_STREAM("Could not fetch transformation from " << trajectory_frame << " to base");
    }
  }

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
