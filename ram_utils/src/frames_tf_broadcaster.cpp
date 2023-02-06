#include <mutex>

#include <eigen_conversions/eigen_msg.h>
#include <ram_utils/GetTrajectoryFrame.h>
#include <ram_utils/GetStartPose.h>
#include <ram_utils/GetTool.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

std::mutex trajectory_frame_mutex;
geometry_msgs::Pose trajectory_frame;

std::mutex start_pose_mutex;
geometry_msgs::Pose start_pose;

std::mutex tool_mutex;
geometry_msgs::Pose tool;

bool getStartPose(ram_utils::GetStartPose::Request &,
                  ram_utils::GetStartPose::Response &res)
{
  std::lock_guard<std::mutex> lock(start_pose_mutex);
  res.pose = start_pose;
  return true;
}

bool getTrajectoryFrame(ram_utils::GetTrajectoryFrame::Request &,
                        ram_utils::GetTrajectoryFrame::Response &res)
{
  std::lock_guard<std::mutex> lock(trajectory_frame_mutex);
  res.pose = trajectory_frame;
  return true;
}

bool getTool(ram_utils::GetTool::Request &,
             ram_utils::GetTool::Response &res)
{
  std::lock_guard<std::mutex> lock(tool_mutex);
  res.pose = tool;
  return true;
}

void updateTrajectoryFrameTFCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(trajectory_frame_mutex);
  trajectory_frame = *msg;
}

void updateStartPoseTFCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(start_pose_mutex);
  start_pose = *msg;
}

void updateToolTFCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(tool_mutex);
  tool = *msg;
}

int main(int argc,
         char** argv)
{
  ros::init(argc, argv, "frames_tf_broadcaster");
  ros::NodeHandle nh;
  std::string base_link;
  nh.param<std::string>("ram/base_link", base_link, "base");

  trajectory_frame.position.x = 0;
  trajectory_frame.position.y = 0;
  trajectory_frame.position.z = 0;
  trajectory_frame.orientation.x = 0;
  trajectory_frame.orientation.y = 0;
  trajectory_frame.orientation.z = 0;
  trajectory_frame.orientation.w = 1;

  start_pose.position.x = 0;
  start_pose.position.y = 0;
  start_pose.position.z = 0;
  start_pose.orientation.x = 0;
  start_pose.orientation.y = 0;
  start_pose.orientation.z = 0;
  start_pose.orientation.w = 1;

  tool.position.x = 0;
  tool.position.y = 0;
  tool.position.z = 0;
  tool.orientation.x = 0;
  tool.orientation.y = 0;
  tool.orientation.z = 0;
  tool.orientation.w = 1;

  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transform_stamped;

  ros::Subscriber sub_1 = nh.subscribe("ram/trajectory_frame", 10, updateTrajectoryFrameTFCallback);
  ros::Subscriber sub_2 = nh.subscribe("ram/start_pose", 10, updateStartPoseTFCallback);
  ros::Subscriber sub_3 = nh.subscribe("ram/tool", 10, updateToolTFCallback);

  ros::ServiceServer server_1 = nh.advertiseService("ram/get_start_pose", getStartPose);
  ros::ServiceServer server_2 = nh.advertiseService("ram/get_trajectory_frame", getTrajectoryFrame);
  ros::ServiceServer server_3 = nh.advertiseService("ram/get_tool", getTool);

  ros::Rate rate(100);
  while (nh.ok())
  {
    // Trajectory frame
    {
      std::lock_guard<std::mutex> lock(trajectory_frame_mutex);
      transform_stamped.header.frame_id = base_link;
      transform_stamped.child_frame_id = "trajectory_frame";
      transform_stamped.transform.translation.x = trajectory_frame.position.x;
      transform_stamped.transform.translation.y = trajectory_frame.position.y;
      transform_stamped.transform.translation.z = trajectory_frame.position.z;
      transform_stamped.transform.rotation = trajectory_frame.orientation;
    }
    transform_stamped.header.stamp = ros::Time::now();
    br.sendTransform(transform_stamped);
    ros::spinOnce();

    // Start pose
    {
      std::lock_guard<std::mutex> lock(start_pose_mutex);
      transform_stamped.header.frame_id = "trajectory_frame";
      transform_stamped.child_frame_id = "start_pose";
      transform_stamped.transform.translation.x = start_pose.position.x;
      transform_stamped.transform.translation.y = start_pose.position.y;
      transform_stamped.transform.translation.z = start_pose.position.z;
      transform_stamped.transform.rotation = start_pose.orientation;
    }
    transform_stamped.header.stamp = ros::Time::now();
    br.sendTransform(transform_stamped);
    ros::spinOnce();

    // Tool orientation
    {
      std::lock_guard<std::mutex> lock(start_pose_mutex);
      transform_stamped.header.frame_id = "start_pose";
      transform_stamped.child_frame_id = "tool_orientation";
      transform_stamped.transform.translation.x = tool.position.x;
      transform_stamped.transform.translation.y = tool.position.y;
      transform_stamped.transform.translation.z = tool.position.z;
      transform_stamped.transform.rotation = tool.orientation;
    }
    transform_stamped.header.stamp = ros::Time::now();
    br.sendTransform(transform_stamped);
    ros::spinOnce();

    rate.sleep();
  }
  return 0;
}
