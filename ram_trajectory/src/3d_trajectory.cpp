#include <eigen_conversions/eigen_msg.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit/planning_interface/planning_interface.h>
#pragma GCC diagnostic pop
#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_trajectory/SimulateTrajectory.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <mutex>

std::string base_link;
std::string group_name;
ram_msgs::AdditiveManufacturingTrajectory trajectory;
std::mutex trajectory_mutex;

void trajectoryCallback(const ram_msgs::AdditiveManufacturingTrajectoryConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(trajectory_mutex);
  trajectory = *msg;
  return;
}

bool simulateTrajectory(ram_trajectory::SimulateTrajectoryRequest &,
                        ram_trajectory::SimulateTrajectoryResponse &res)
{
  res.error.clear();
  if (trajectory.poses.empty())
  {
    res.error = "Trajectory is empty";
    return true;
  }

  tf::TransformListener listener;
  tf::StampedTransform start_pose_from_base;
  tf::StampedTransform tool_orientation_from_start_pose;
  try
  {
    listener.waitForTransform(base_link, "start_pose", ros::Time(0), ros::Duration(10.0));
    listener.lookupTransform(base_link, "start_pose", ros::Time(0), start_pose_from_base);
    listener.waitForTransform("start_pose", "tool_orientation", ros::Time(0), ros::Duration(10.0));
    listener.lookupTransform("start_pose", "tool_orientation", ros::Time(0), tool_orientation_from_start_pose);
  }
  catch (tf::TransformException &ex)
  {
    res.error = "Could not get transforms: ";
    res.error.append(ex.what());
    return true;
  }

  Eigen::Isometry3d start_pose(Eigen::Isometry3d::Identity());
  tf::transformTFToEigen(start_pose_from_base, start_pose);
  Eigen::Isometry3d tool_orientation(Eigen::Isometry3d::Identity());
  tf::transformTFToEigen(tool_orientation_from_start_pose, tool_orientation);

  std::vector<geometry_msgs::Pose> way_points_msg;
  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > way_points_vector;
  way_points_msg.resize(trajectory.poses.size());
  for (unsigned i(0); i < trajectory.poses.size(); i++)
  {
    way_points_msg[i] = trajectory.poses[i].pose;
    way_points_vector.resize(way_points_msg.size());
    tf::poseMsgToEigen(way_points_msg[i], way_points_vector[i]);
    Eigen::Affine3d base(Eigen::Affine3d::Identity());
    Eigen::Affine3d to_pose(Eigen::Affine3d::Identity());
    to_pose.translation() = way_points_vector[i].translation() - base.translation();
    Eigen::Affine3d to_origin(Eigen::Affine3d::Identity());
    to_origin.translation() = base.translation() - way_points_vector[i].translation();
    way_points_vector[i] = to_origin * way_points_vector[i];
    way_points_vector[i] = tool_orientation * way_points_vector[i];
    way_points_vector[i] = to_pose * way_points_vector[i];
    way_points_vector[i] = start_pose * way_points_vector[i];
    tf::poseEigenToMsg(way_points_vector[i], way_points_msg[i]);
  }

  moveit::planning_interface::MoveGroupInterface group(group_name);
  group.setPoseReferenceFrame(base_link);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  double result(group.computeCartesianPath(way_points_msg, 0.05, 15, plan.trajectory_));
  if (result == -1)
  {
    res.error = "Path planning failed";
    return true;
  }

  if (!group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    res.error = "Could not execute planned trajectory";
    return true;
  }

  if (result < 1)
    res.error = "Path planning only could plan for " + std::to_string(result) + "% of the trajectory";
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulate_trajectory");
  ros::NodeHandle nh;
  nh.param<std::string>("ram/base_link", base_link, "base");
  nh.param<std::string>("ram/group_name", group_name, "");
  ros::AsyncSpinner spinner(3); // At least 2: One for the callback, one for MoveIt
  spinner.start();

  ros::ServiceServer server = nh.advertiseService("ram_trajectory/simulate", simulateTrajectory);
  ros::Subscriber sub = nh.subscribe("ram/trajectory", 10, trajectoryCallback);
  ros::waitForShutdown();
  return 0;
}
