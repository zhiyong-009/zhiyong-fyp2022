#include <eigen_conversions/eigen_msg.h>
#include <math.h>
#include <mutex>
#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_utils/AddEntryExitStrategies.h>
#include <ram_utils/EntryExitParameters.h>
#include <ros/ros.h>
#include <unique_id/unique_id.h>

ram_utils::EntryExitParametersRequest entry_params;
ram_utils::EntryExitParametersRequest exit_params;

std::mutex params_mutex;

bool entryParametersCallback(ram_utils::EntryExitParameters::Request &req,
                             ram_utils::EntryExitParameters::Response &)
{
  std::lock_guard<std::mutex> lock(params_mutex);
  entry_params = req;
  return true;
}

bool exitParametersCallback(ram_utils::EntryExitParameters::Request &req,
                            ram_utils::EntryExitParameters::Response &)
{
  std::lock_guard<std::mutex> lock(params_mutex);
  exit_params = req;
  return true;
}

bool entryStrategy(unsigned ref_pose_id,
                   std::vector<ram_msgs::AdditiveManufacturingPose> req_poses,
                   std::vector<ram_msgs::AdditiveManufacturingPose> &res_poses)
{
  Eigen::Vector3d ref_vector(Eigen::Vector3d::Identity());

  if (req_poses.size() != 1)
  {
    ref_vector.x() = req_poses[ref_pose_id].pose.position.x - req_poses[ref_pose_id + 1].pose.position.x;
    ref_vector.y() = req_poses[ref_pose_id].pose.position.y - req_poses[ref_pose_id + 1].pose.position.y;
    ref_vector.z() = req_poses[ref_pose_id].pose.position.z - req_poses[ref_pose_id + 1].pose.position.z;
  }

  double rho;
  double phi;
  double theta;

  ram_msgs::AdditiveManufacturingPose pose(req_poses[ref_pose_id]);
  pose.params.laser_power = 0;
  pose.params.feed_rate = 0;
  pose.entry_pose = true;
  pose.polygon_start = false;
  pose.polygon_end = false;
  pose.params.movement_type = entry_params.params.movement_type;
  pose.params.approach_type = entry_params.params.approach_type;
  pose.params.blend_radius = entry_params.params.blend_radius;
  pose.params.speed = entry_params.params.non_printing_move_speed;

  phi = atan2(ref_vector.y(), ref_vector.x());
  theta = M_PI / 2 - entry_params.angle;

  for (unsigned j(entry_params.number_of_poses); j > 0; --j)
  {
    rho = entry_params.distance * j / entry_params.number_of_poses;

    ram_msgs::AdditiveManufacturingPose pose_tmp(pose);
    //Spherical to Cartesian coordinates
    pose_tmp.pose.position.x += rho * sin(theta) * cos(phi);
    pose_tmp.pose.position.y += rho * sin(theta) * sin(phi);
    pose_tmp.pose.position.z += rho * cos(theta);

    pose_tmp.unique_id = unique_id::toMsg(unique_id::fromRandom());
    res_poses.push_back(pose_tmp);
  }
  return true;
}

bool exitStrategy(unsigned ref_pose_id,
                  std::vector<ram_msgs::AdditiveManufacturingPose> req_poses,
                  std::vector<ram_msgs::AdditiveManufacturingPose> &res_poses)
{
  Eigen::Vector3d ref_vector(Eigen::Vector3d::Identity());

  // Make sure it is not the only pose
  // Make sure it is not the first pose
  if (req_poses.size() != 1 && ref_pose_id != 0)
  {
    ref_vector.x() = req_poses[ref_pose_id].pose.position.x - req_poses[ref_pose_id - 1].pose.position.x;
    ref_vector.y() = req_poses[ref_pose_id].pose.position.y - req_poses[ref_pose_id - 1].pose.position.y;
    ref_vector.z() = req_poses[ref_pose_id].pose.position.z - req_poses[ref_pose_id - 1].pose.position.z;
  }
  double rho;
  double phi;
  double theta;

  ram_msgs::AdditiveManufacturingPose pose(req_poses[ref_pose_id]);
  pose.params.laser_power = 0;
  pose.params.feed_rate = 0;
  pose.exit_pose = true;
  pose.polygon_start = false;
  pose.polygon_end = false;
  pose.params.movement_type = exit_params.params.movement_type;
  pose.params.approach_type = exit_params.params.approach_type;
  pose.params.blend_radius = exit_params.params.blend_radius;
  pose.params.speed = exit_params.params.non_printing_move_speed;

  phi = atan2(ref_vector.y(), ref_vector.x());
  theta = M_PI / 2 - exit_params.angle;

  for (unsigned j(1); j <= exit_params.number_of_poses; ++j)
  {
    rho = exit_params.distance * j / exit_params.number_of_poses;

    ram_msgs::AdditiveManufacturingPose pose_tmp(pose);
    //Spherical to Cartesian coordinates
    pose_tmp.pose.position.x += rho * sin(theta) * cos(phi);
    pose_tmp.pose.position.y += rho * sin(theta) * sin(phi);
    pose_tmp.pose.position.z += rho * cos(theta);

    pose_tmp.unique_id = unique_id::toMsg(unique_id::fromRandom());
    res_poses.push_back(pose_tmp);
  }
  return true;
}

bool poseInPolygon(std::vector<ram_msgs::AdditiveManufacturingPose> poses, unsigned pose_id, bool &in_polygon)
{
  int polygon_start(0);

  for (std::size_t i(0); i < poses.size(); ++i)
  {
    if (poses[i].polygon_start)
      polygon_start++;

    if (poses[i].polygon_end)
      polygon_start--;

    if (i == pose_id)
    {
      in_polygon = !(polygon_start > 1 || polygon_start < 0);
      return true;
    }
  }
  in_polygon = false;
  return false;
}

bool addEntryExitStrategiesCallback(ram_utils::AddEntryExitStrategies::Request &req,
                                    ram_utils::AddEntryExitStrategies::Response &res)
{
  std::lock_guard<std::mutex> lock(params_mutex);

  for (unsigned i(0); i < req.poses.size(); ++i)
  {
    bool limit_between_polygons = false;
    bool add_entry_strategy = false;
    bool add_exit_strategy = false;
    bool in_polygon = false;

    // Special case : limit between two polygons
    if (req.poses[i].polygon_start && req.poses[i].polygon_end)
    {
      bool pose_in_polygon(poseInPolygon(req.poses, i, in_polygon));

      if (!in_polygon || !pose_in_polygon)
      {
        res.error = "Multiple adjacent polygon start/end in trajectory \nPose does not exist in the trajectory";
        return true;
      }
    }

    // Search for polygon_start and polygon_end
    // Specific case : if a polygon contains only one pose
    // This pose contains the entry and exit pose
    if (req.poses[i].polygon_start && req.poses[i].polygon_end)
    {
      // Add Entry Strategy and Exit Strategy on the same pose
      add_entry_strategy = true;
      add_exit_strategy = true;
    }
    else
    {
      // find polygon_start
      if (req.poses[i].polygon_start)
      {
        // Add Entry Strategy
        if (i == 0) //First Element
          add_entry_strategy = true;
        if (i != 0 && !req.poses[i - 1].entry_pose)
          add_entry_strategy = true;
      }

      // find polygon_end
      if (req.poses[i].polygon_end)
      {
        // Add Exit Pose
        if (i == (req.poses.size() - 1)) //Last Element
          add_exit_strategy = true;
        if (i != (req.poses.size() - 1) && !req.poses[i + 1].exit_pose)
          add_exit_strategy = true;
      }
    }

    // Add entry and exit strategies
    // Specific case : if a polygon contains only one pose
    // This pose contains the entry and exit pose
    // First, is it the specific case only for Follow poses path planning ?
    if (req.poses[i].polygon_start && req.poses[i].polygon_end)
    {
      entryStrategy(i, req.poses, res.poses);
      res.poses.push_back(req.poses[i]);
      exitStrategy(i, req.poses, res.poses);
    }
    else // For other cases
    {
      if (limit_between_polygons)
      {
        res.poses.push_back(req.poses[i]);
        res.poses.back().polygon_start = false;

        exitStrategy(i, req.poses, res.poses);

        entryStrategy(i, req.poses, res.poses);

        res.poses.push_back(req.poses[i]);
        res.poses.back().polygon_end = false;
        res.poses.back().unique_id = unique_id::toMsg(unique_id::fromRandom());
      }
      else
      {
        if (add_entry_strategy)
          entryStrategy(i, req.poses, res.poses);

        res.poses.push_back(req.poses[i]);

        if (add_exit_strategy)
          exitStrategy(i, req.poses, res.poses);
      }
    }
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "entry_exit_strategies");
  ros::NodeHandle nh;

  entry_params.angle = M_PI / 4; // 45 degrees (phi in spherical coordinates)
  entry_params.distance = 0.05; // 5 cm
  entry_params.number_of_poses = 1;
  entry_params.params.movement_type = 1;
  entry_params.params.approach_type = 1;
  entry_params.params.blend_radius = 100;
  entry_params.params.non_printing_move_speed = 0.01;
  entry_params.params.printing_move_speed = 0.01;

  exit_params.angle = M_PI / 4; // 45 degrees (phi in spherical coordinates)
  exit_params.distance = 0.05; // 5 cm
  exit_params.number_of_poses = 1;
  exit_params.params.movement_type = 1;
  exit_params.params.approach_type = 1;
  exit_params.params.blend_radius = 100;
  exit_params.params.non_printing_move_speed = 0.01;
  exit_params.params.printing_move_speed = 0.01;

  ros::ServiceServer service_1 = nh.advertiseService("ram/information/add_entry_exit_strategies",
                                                     addEntryExitStrategiesCallback);

  ros::ServiceServer service_2 = nh.advertiseService("ram/information/entry_parameters", entryParametersCallback);
  ros::ServiceServer service_3 = nh.advertiseService("ram/information/exit_parameters", exitParametersCallback);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
