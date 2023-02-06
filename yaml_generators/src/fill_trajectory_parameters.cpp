#include <mutex>
#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_utils/AddEntryExitStrategies.h>
#include <ram_utils/GetFillParameters.h>
#include <ros/ros.h>

std::mutex params_mutex;
ram_msgs::FillParameters params;
std::mutex trajectory_mutex;
ram_msgs::AdditiveManufacturingTrajectory trajectory;

ros::ServiceClient entry_exit_strategies_client;
ros::Publisher pub_trajectory;

void publishModifiedTrajectory()
{
  if (trajectory.poses.empty())
    return;

  // Add exit and entry Strategies
  ram_utils::AddEntryExitStrategies srv;
  srv.request.poses = trajectory.poses;

  if (!entry_exit_strategies_client.call(srv))
    ROS_INFO_STREAM("Failed to call " + entry_exit_strategies_client.getService() + " service");

  if (!srv.response.error.empty())
    ROS_INFO_STREAM(srv.response.error);

  trajectory.poses = srv.response.poses;

  double next_speed = params.non_printing_move_speed;

  ram_msgs::AdditiveManufacturingPose last_pose(trajectory.poses.at(0));
  for (auto &pose : trajectory.poses)
  {
    if (pose.entry_pose || pose.exit_pose)
      continue;

    pose.params.approach_type = params.approach_type;
    pose.params.blend_radius = params.blend_radius;
    pose.params.feed_rate = params.feed_rate;
    pose.params.laser_power = params.laser_power;
    pose.params.movement_type = params.movement_type;
    pose.params.speed = next_speed;

    if (pose.polygon_start)
    {
      pose.params = last_pose.params;
      next_speed = params.printing_move_speed;
    }
    else if (pose.polygon_end)
    {
      next_speed = params.non_printing_move_speed;
    }

    last_pose = pose;
  }

  // Now that the trajectory is complete, we set the time stamp to now and publish
  trajectory.generated = ros::Time::now();
  trajectory.modified = trajectory.generated;
  pub_trajectory.publish(trajectory);
}

void updateParams(const ram_msgs::FillParameters::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock_1(params_mutex);
  std::lock_guard<std::mutex> lock_2(trajectory_mutex);
  params = *msg;
}

void tempTrajReceived(const ram_msgs::AdditiveManufacturingTrajectory::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock_1(params_mutex);
  std::lock_guard<std::mutex> lock_2(trajectory_mutex);
  trajectory = *msg;
  publishModifiedTrajectory();
}

bool getFillParams(ram_utils::GetFillParametersRequest &, ram_utils::GetFillParametersResponse &res)
{
  std::lock_guard<std::mutex> lock(params_mutex);
  res.parameters = params;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fill_trajectory_parameters");
  ros::NodeHandle nh;

  // Default parameters
  params.approach_type = 1;
  params.laser_power = 4000;
  params.feed_rate = 2.0 / 60.0; // Wire - 2 meters / minute
  params.movement_type = 1;
  params.blend_radius = 0;
  // Speeds in meters / second:
  params.printing_move_speed = 0.01;
  params.non_printing_move_speed = 0.01;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Subscribe to update internal parameters
  ros::Subscriber sub_params = nh.subscribe("ram/fill_trajectory/parameters", 5, updateParams);

  // Subscribe to receive trajectory from path_planning
  ros::Subscriber sub_traj = nh.subscribe("ram/trajectory_tmp", 5, tempTrajReceived);

  // Publish "ram/trajectory" to every node that listens
  pub_trajectory = nh.advertise<ram_msgs::AdditiveManufacturingTrajectory>("ram/trajectory", 5, true);

  // Service to add strategies
  entry_exit_strategies_client = nh.serviceClient<ram_utils::AddEntryExitStrategies>(
      "ram/information/add_entry_exit_strategies");

// Get fill parameters stored internally
  ros::ServiceServer server = nh.advertiseService("ram/fill_trajectory/get_parameters", getFillParams);

  ros::waitForShutdown();
  spinner.stop();
  return 0;
}

