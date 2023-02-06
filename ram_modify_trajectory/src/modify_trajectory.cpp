#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#pragma GCC diagnostic pop
#include <iterator>
#include <mutex>
#include <ram_modify_trajectory/AddPoses.h>
#include <ram_modify_trajectory/ChangeLayerHeight.h>
#include <ram_modify_trajectory/DeleteSelectedPoses.h>
#include <ram_modify_trajectory/ModifySelectedPoses.h>
#include <ram_modify_trajectory/PushPullAngle.h>
#include <ram_modify_trajectory/ReflectSelectedPoses.h>
#include <ram_modify_trajectory/ResetSelectedPoses.h>
#include <ram_modify_trajectory/RotateSelectedPoses.h>
#include <ram_modify_trajectory/ScaleSelectedPoses.h>
#include <ram_modify_trajectory/ShiftPoses.h>
#include <ram_modify_trajectory/SimplifyTrajectory.h>
#include <ram_modify_trajectory/TrajectoryInterruption.h>
#include <ram_modify_trajectory/TweakBlendRadius.h>
#include <ram_msgs/AdditiveManufacturingPose.h>
#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_msgs/AdditiveManufacturingTrajectoryInfo.h>
#include <ram_utils/AddEntryExitStrategies.h>
#include <ram_utils/UnmodifiedTrajectory.h>
#include <ros/ros.h>
#include <string>
#include <strings.h>
#include <unique_id/unique_id.h>
#include <uuid_msgs/UniqueID.h>

ros::Publisher trajectory_pub;
ros::ServiceClient unmodified_trajectory_client;
ros::ServiceClient entry_exit_strategies_client;

std::mutex layers_mutex;
ram_msgs::AdditiveManufacturingTrajectory layers;
std::mutex info_mutex;
ram_msgs::AdditiveManufacturingTrajectoryInfo info_trajectory;

void verifyPolygonLimits(ram_msgs::AdditiveManufacturingTrajectory trajectory)
{
  bool pose_in_polygon = false;

  for (auto current_pose : trajectory.poses)
  {
    if ((current_pose.polygon_start || current_pose.polygon_end) && (current_pose.entry_pose || current_pose.exit_pose))
      throw std::runtime_error("Wrong trajectory start");

    if (current_pose.polygon_start && current_pose.polygon_end)
      continue;

    if (current_pose.polygon_start)
    {
      if (!pose_in_polygon)
        pose_in_polygon = true;
      else
      {
        std::stringstream stream;
        for (auto & uuid : current_pose.unique_id.uuid)
          stream << std::hex << (unsigned) uuid;
        const std::string uuid_str = stream.str();
        throw std::runtime_error(std::string("Wrong polygon_start at pose UUID " + uuid_str));
      }
    }
    else if (current_pose.polygon_end)
    {
      if (pose_in_polygon)
        pose_in_polygon = false;
      else
      {
        std::stringstream stream;
        for (auto & uuid : current_pose.unique_id.uuid)
          stream << std::hex << (unsigned) uuid;
        const std::string uuid_str = stream.str();
        throw std::runtime_error(std::string("Wrong polygon_end at pose UUID " + uuid_str));
      }
    }
  }

  if (pose_in_polygon)
  {
    throw std::runtime_error("Missing polygon_end at the end of the trajectory");
  }
}

bool deletePoses(ram_msgs::AdditiveManufacturingTrajectory &trajectory,
                 std::vector<ram_msgs::AdditiveManufacturingPose> poses)
{
  for (auto &pose : poses)
  {
    bool pose_is_deleted = false;

    // Poses in the trajectory
    for (std::size_t j(0); j < trajectory.poses.size(); ++j)
    {
      if (pose.unique_id.uuid != trajectory.poses[j].unique_id.uuid) // uuid are not equals
        continue;

      trajectory.poses.erase(trajectory.poses.begin() + j); // Delete pose
      pose_is_deleted = true;
      break;
    }

    if (!pose_is_deleted) // Selected pose is not in the trajectory
    {
      ROS_ERROR_STREAM("Pose to be deleted could not be found, UUID = " << unique_id::toHexString(pose.unique_id));
      return false;
    }
  }

  trajectory.similar_layers = false;
  return true;
}

bool findPose(const boost::array<uint8_t, 16> &pose_uuid, std::vector<ram_msgs::AdditiveManufacturingPose> poses,
              ram_msgs::AdditiveManufacturingPose &pose)
{
  for (auto current_pose : poses)
  {
    if (pose_uuid != current_pose.unique_id.uuid) // uuid are not equals
      continue;

    // uuid are equals
    pose = current_pose;
    return true;
  }
  return false;
}

void trajectoryCallback(const ram_msgs::AdditiveManufacturingTrajectoryConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(layers_mutex);
  layers = *msg;
}

void trajectoryInfoCallback(const ram_msgs::AdditiveManufacturingTrajectoryInfoConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(info_mutex);
  info_trajectory = *msg;
}

bool modifySelectedPosesCallback(ram_modify_trajectory::ModifySelectedPoses::Request &req,
                                 ram_modify_trajectory::ModifySelectedPoses::Response &res)
{
  std::lock_guard<std::mutex> lock(layers_mutex);

  if (req.poses.empty())
  {
    res.error = "Request is empty";
    return true;
  }

  // Make a copy and modify this copy
  ram_msgs::AdditiveManufacturingTrajectory trajectory(layers);

  if (trajectory.poses.empty())
  {
    res.error = "Trajectory is empty";
    return true;
  }

  std::vector<ram_msgs::AdditiveManufacturingParams::_movement_type_type> move_types;
  for (auto pose : req.poses)
    move_types.push_back(pose.params.movement_type);

  if (std::adjacent_find(move_types.begin(), move_types.end(), std::not_equal_to<>()) != move_types.end())
  {
    // Cannot modify speed if not all the poses have the same motion type
    if (req.speed)
    {
      res.error = "Cannot modify speed if not all the poses have the same motion type";
      return true;
    }
  }

  // Strategies to delete when a polygon_start or a polygon_end is modified
  std::vector<ram_msgs::AdditiveManufacturingPose> strategies_to_delete;
  // Modify poses
  for (std::size_t i(0); i < req.poses.size(); ++i)
  {
    bool pose_in_trajectory = false;

    for (std::size_t j(0); j < trajectory.poses.size(); ++j)
    {
      if (req.poses[i].unique_id.uuid != trajectory.poses[j].unique_id.uuid) // uuid are not equals
        continue;

      // uuid are equals. Modify this pose
      pose_in_trajectory = true;

      // Replace and check values
      if (req.pose)
        trajectory.poses[j].pose = req.pose_reference.pose;
      else
      {
        Eigen::Affine3d current_pose_eigen, reference_pose;
        tf::poseMsgToEigen(trajectory.poses[j].pose, current_pose_eigen);
        tf::poseMsgToEigen(req.pose_reference.pose, reference_pose);
        current_pose_eigen.linear() *= reference_pose.linear();
        current_pose_eigen.translation() += reference_pose.translation();
        tf::poseEigenToMsg(current_pose_eigen, trajectory.poses[j].pose);
      }

      // Non numerical values
      if (req.polygon_start)
      {
        if (trajectory.poses[j].polygon_start && !req.pose_reference.polygon_start)
        {
          // Save the entry strategy in the vector
          for (int strategy_id(j - 1); strategy_id >= 0; --strategy_id)
          {
            if (trajectory.poses[strategy_id].entry_pose)
              strategies_to_delete.push_back(trajectory.poses[strategy_id]);
            else
              break;
          }
        }

        trajectory.poses[j].polygon_start = req.pose_reference.polygon_start;
      }

      if (req.polygon_end)
      {
        // Save the exit strategy in the vector
        if (trajectory.poses[j].polygon_end && !req.pose_reference.polygon_end)
        {
          for (std::size_t strategy_id(j + 1); strategy_id < trajectory.poses.size(); ++strategy_id)
          {
            if (trajectory.poses[strategy_id].exit_pose)
              strategies_to_delete.push_back(trajectory.poses[strategy_id]);
            else
              break;
          }
        }
        trajectory.poses[j].polygon_end = req.pose_reference.polygon_end;
      }

      if (req.movement_type)
        trajectory.poses[j].params.movement_type = req.pose_reference.params.movement_type;

      if (req.approach_type)
        trajectory.poses[j].params.approach_type = req.pose_reference.params.approach_type;

      // Numerical values
      if (req.blend_radius)
        trajectory.poses[j].params.blend_radius = req.pose_reference.params.blend_radius;
      else
      {
        trajectory.poses[j].params.blend_radius += req.pose_reference.params.blend_radius;

        if (trajectory.poses[j].params.blend_radius < 0)
        {
          res.error = "Blend radius cannot be < 0";
          return true;
        }

        if (trajectory.poses[j].params.blend_radius > 100)
        {
          res.error = "Blend radius cannot be > 100";
          return true;
        }
      }

      if (req.speed)
        trajectory.poses[j].params.speed = req.pose_reference.params.speed;
      else
      {
        trajectory.poses[j].params.speed += req.pose_reference.params.speed;

        if (trajectory.poses[j].params.speed < 1e-12) // Cannot be zero
        {
          res.error = "Robot speed cannot be < 1e-12";
          return true;
        }
      }

      if (req.laser_power)
        trajectory.poses[j].params.laser_power = req.pose_reference.params.laser_power;
      else
      {
        trajectory.poses[j].params.laser_power += req.pose_reference.params.laser_power;

        if (trajectory.poses[j].params.laser_power < 0)
        {
          res.error = "Laser power cannot be < 0";
          return true;
        }
      }

      if (req.feed_rate)
        trajectory.poses[j].params.feed_rate = req.pose_reference.params.feed_rate;
      else
      {
        trajectory.poses[j].params.feed_rate += req.pose_reference.params.feed_rate;

        if (trajectory.poses[j].params.feed_rate < 0)
        {
          res.error = "Feed rate cannot be < 0";
          return true;
        }
      }

      break; // Exit to for loop
    }
    if (!pose_in_trajectory)
    {
      res.error = "There are pose in the request that are not in the trajectory";
      return true;
    }
  }

  // Delete the strategies
  if (!deletePoses(trajectory, strategies_to_delete))
  {
    res.error = "Error deleting strategies_to_delete";
    return true;
  }

  // Verify polygon_start and polygon_end
  try
  {
    verifyPolygonLimits(trajectory);
  }
  catch (std::runtime_error &e)
  {
    res.error = "Error in verifyPolygonLimits: " + std::string(e.what());
    return true;
  }

  // Add exit and entry strategies
  ram_utils::AddEntryExitStrategies srv;
  srv.request.poses = trajectory.poses;

  if (!entry_exit_strategies_client.call(srv))
  {
    res.error = "Failed to call " + entry_exit_strategies_client.getService() + " service";
    return true;
  }

  if (!srv.response.error.empty())
  {
    res.error = srv.response.error;
    return true;
  }

  trajectory.poses = srv.response.poses;

  // Publish trajectory
  trajectory.modified = ros::Time::now();
  trajectory_pub.publish(trajectory);

  return true;
}

// Reset the poses in the trajectory with the poses in the unmodified trajectory.
// the service request contains a list of uuid
bool resetSelectedPosesCallback(ram_modify_trajectory::ResetSelectedPoses::Request &req,
                                ram_modify_trajectory::ResetSelectedPoses::Response &res)
{
  std::lock_guard<std::mutex> lock(layers_mutex);
  // Make a copy and modify this copy
  ram_msgs::AdditiveManufacturingTrajectory trajectory(layers);

  if (req.poses.empty())
  {
    res.error = "Request is empty";
    return true;
  }

  if (trajectory.poses.empty())
  {
    res.error = "Trajectory is empty";
    return true;
  }

  //Call buffer service
  ram_utils::UnmodifiedTrajectory unmodified_trajectory;
  unmodified_trajectory.request.generated = trajectory.generated;
  if (!unmodified_trajectory_client.call(unmodified_trajectory))
  {
    res.error = "Cannot call unmodified_trajectory_client with unmodified_trajectory";
    return true;
  }

  // Reset poses
  // - Poses in the current layer added after the generation
  // - Poses in the strategies to delete
  std::vector<ram_msgs::AdditiveManufacturingPose> poses_to_delete;

  for (auto &pose : req.poses)
  {
    bool pose_in_current_traj = false;
    for (std::size_t j(0); j < trajectory.poses.size(); ++j)
    {
      if (trajectory.poses[j].unique_id.uuid != pose.unique_id.uuid) // uuid are not equals
        continue;

      // uuid are equals
      pose_in_current_traj = true;
      //Find pose in unmodified trajectory
      ram_msgs::AdditiveManufacturingPose unmodified_pose;
      ram_msgs::AdditiveManufacturingPose p;

      if (findPose(trajectory.poses[j].unique_id.uuid, unmodified_trajectory.response.trajectory.poses,
                   unmodified_pose))
      {
        // Save the entry strategy in the vector
        if (trajectory.poses[j].polygon_start && !unmodified_pose.polygon_start)
        {
          // Save the entry strategy in the vector
          for (int strategy_id(j - 1); strategy_id >= 0; --strategy_id)
          {
            if (trajectory.poses[strategy_id].entry_pose)
            {
              if (!findPose(trajectory.poses[j].unique_id.uuid, poses_to_delete, p))
                poses_to_delete.push_back(trajectory.poses[strategy_id]);
            }
            else
              break;
          }
        }
        // Save the exit strategy in the vector
        for (std::size_t strategy_id(j + 1); strategy_id < trajectory.poses.size(); ++strategy_id)
        {
          if (trajectory.poses[strategy_id].exit_pose)
          {
            if (!findPose(trajectory.poses[j].unique_id.uuid, poses_to_delete, p))
              poses_to_delete.push_back(trajectory.poses[strategy_id]);
          }
          else
            break;
        }
        //update pose value
        trajectory.poses[j] = unmodified_pose;
      }
      else
      {
        if (!findPose(trajectory.poses[j].unique_id.uuid, poses_to_delete, p))
          poses_to_delete.push_back(
              trajectory.poses[j]); // Pose in request service does not exist in unmodified trajectory
      }

      break;
    }
    if (!pose_in_current_traj) // Pose in the request that are not in the current trajectory
    {
      res.error = "Pose in the request that are not in the current trajectory";
      return true;
    }
  }
  //Delete poses
  if (!deletePoses(trajectory, poses_to_delete))
  {
    res.error = "Cannot delete pose";
    return true;
  }

  // Verify polygon_start and polygon_end
  try
  {
    verifyPolygonLimits(trajectory);
  }
  catch (std::runtime_error &e)
  {
    res.error = std::string("Error in verifyPolygonLimits: " + std::string(e.what()));
    return true;
  }

  // Add exit and entry Strategies
  ram_utils::AddEntryExitStrategies srv;
  srv.request.poses = trajectory.poses;

  if (!entry_exit_strategies_client.call(srv))
  {
    res.error = "Failed to call " + entry_exit_strategies_client.getService() + " service";
    return true;
  }

  if (!srv.response.error.empty())
  {
    res.error = srv.response.error;
    return true;
  }

  trajectory.poses = srv.response.poses;

// Publish trajectory
  trajectory.modified = ros::Time::now();
  trajectory_pub.publish(trajectory);

  return true;
}

// Add a new pose before each pose in the request
bool addPosesCallback(ram_modify_trajectory::AddPoses::Request &req,
                      ram_modify_trajectory::AddPoses::Response &res)
{
  std::lock_guard<std::mutex> lock(layers_mutex);

  // Make a copy and modify this copy
  ram_msgs::AdditiveManufacturingTrajectory trajectory(layers);

  if (req.poses.empty())
  {
    res.error = "Request is empty";
    return true;
  }

  if (trajectory.poses.empty())
  {
    res.error = "Trajectory is empty";
    return true;
  }

  // Find the selected pose
  for (auto &pose : req.poses)
  {
    bool pose_in_trajectory = false;

    for (std::size_t j(0); j < trajectory.poses.size(); ++j)
    {
      if (pose.unique_id.uuid != trajectory.poses[j].unique_id.uuid) // uuid are not equals
        continue;

      // uuid are equals. Add new pose
      pose_in_trajectory = true;
      ram_msgs::AdditiveManufacturingPose new_pose = trajectory.poses[j];

      new_pose.unique_id = unique_id::toMsg(unique_id::fromRandom());

      new_pose.polygon_start = false;
      new_pose.polygon_end = false;

      // Exit strategy
      if (trajectory.poses[j].polygon_end)
        trajectory.poses[j].exit_pose = true;

      if ((j + 1) == trajectory.poses.size()) // Last element in the trajectory
      {
        trajectory.poses.push_back(new_pose); // Insert new pose
      }
      else
      {
        // Modify position values
        new_pose.pose.position.x = (trajectory.poses[j].pose.position.x + trajectory.poses[j + 1].pose.position.x) / 2;
        new_pose.pose.position.y = (trajectory.poses[j].pose.position.y + trajectory.poses[j + 1].pose.position.y) / 2;
        new_pose.pose.position.z = (trajectory.poses[j].pose.position.z + trajectory.poses[j + 1].pose.position.z) / 2;

        trajectory.poses.insert(trajectory.poses.begin() + j + 1, new_pose); // Insert new pose
      }
      break;
    }
    if (!pose_in_trajectory)
    {
      res.error = "Pose in the request that are not in the trajectory";
      return true;
    }
  }

  trajectory.similar_layers = false;
  trajectory.modified = ros::Time::now();
  trajectory_pub.publish(trajectory);
  return true;
}

bool deleteSelectedPosesCallback(ram_modify_trajectory::DeleteSelectedPoses::Request &req,
                                 ram_modify_trajectory::DeleteSelectedPoses::Response &res)
{
  std::lock_guard<std::mutex> lock(layers_mutex);
  // Make a copy and modify this copy
  ram_msgs::AdditiveManufacturingTrajectory trajectory(layers);

  if (req.poses.empty())
  {
    res.error = "Request is empty";
    return true;
  }

  if (trajectory.poses.empty())
  {
    res.error = "Trajectory is empty";
    return true;
  }

  std::vector<ram_msgs::AdditiveManufacturingPose> strategies_to_delete; // Strategies to delete when a polygon_start or a polygon_end is modified

  // Delete poses
  for (auto &pose : req.poses)
  {
    bool pose_is_deleted = false;

    // Poses in the trajectory
    for (std::size_t j(0); j < trajectory.poses.size(); ++j)
    {
      if (pose.unique_id.uuid != trajectory.poses[j].unique_id.uuid) // uuid are not equals
        continue;

      // uuid are equals. Delete pose
      ram_msgs::AdditiveManufacturingPose p;

      if (trajectory.poses[j].polygon_start)
      {
        // We also want to delete the entry strategy
        for (int strategy_id(j - 1); strategy_id >= 0; --strategy_id)
        {
          if (trajectory.poses[strategy_id].entry_pose)
          {
            // Do not add the pose if it is already in the req.poses vector: it will be deleted
            // This avoids duplicates between the strategies_to_delete vector and req.poses
            if (!findPose(trajectory.poses[strategy_id].unique_id.uuid, req.poses, p))
              if (!findPose(trajectory.poses[strategy_id].unique_id.uuid, strategies_to_delete, p))
                // Avoid duplicates in the strategies to delete
                strategies_to_delete.push_back(trajectory.poses[strategy_id]);
          }
          else
            break;
        }
      }

      if (trajectory.poses[j].polygon_end)
      {
        // We also want to delete the exit strategy
        for (std::size_t strategy_id(j + 1); strategy_id < trajectory.poses.size(); ++strategy_id)
        {
          if (trajectory.poses[strategy_id].exit_pose)
          {
            // Do not add the pose if it is already in the req.poses vector: it will be deleted
            // This avoids duplicates between the strategies_to_delete vector and req.poses
            if (!findPose(trajectory.poses[strategy_id].unique_id.uuid, req.poses, p))
              if (!findPose(trajectory.poses[strategy_id].unique_id.uuid, strategies_to_delete, p))
                // Avoid duplicates in the strategies to delete
                strategies_to_delete.push_back(trajectory.poses[strategy_id]);
          }
          else
            break;
        }
      }

      if (trajectory.poses[j].polygon_start && !trajectory.poses[j].polygon_end) // Verify polygon_start value
      {
        if ((j + 1) != trajectory.poses.size()) // Is not the last pose
          trajectory.poses[j + 1].polygon_start = true; // The next pose is the new polygon_start
      }
      else if (trajectory.poses[j].polygon_end && !trajectory.poses[j].polygon_start) // Verify polygon_end value
      {
        if (j != 0) // Is not the first pose
          trajectory.poses[j - 1].polygon_end = true; // The previous pose is the new polygon_end
      }
      // If pose is a polygon start AND a polygon end, do nothing

      trajectory.poses.erase(trajectory.poses.begin() + j); // Delete pose
      pose_is_deleted = true;
      break;
    }

    if (!pose_is_deleted)
    {
      res.error = "Selected pose are not in the trajectory";
      return true;
    }
  }

  if (trajectory.poses.empty())
  {
    res.error = "Trajectory is empty";
    return true;
  }

  // Delete poses
  if (!deletePoses(trajectory, strategies_to_delete))
  {
    res.error = "Cannot delete poses";
    return true;
  }

  if (trajectory.poses.empty())
  {
    res.error = "Trajectory is empty";
    return true;
  }

  bool trajectory_ok(false);
  for (auto pose : trajectory.poses)
  {
    if (!pose.entry_pose && !pose.exit_pose)
    {
      trajectory_ok = true;
      break;
    }
  }

  if (!trajectory_ok)
  {
    res.error = "Trajectory contains errors related to polygon_start/end or entry/exit poses";
    return true;
  }

  // Verify polygon_start and polygon_end
  try
  {
    verifyPolygonLimits(trajectory);
  }
  catch (std::runtime_error &e)
  {
    res.error = std::string("Error in verifyPolygonLimits: " + std::string(e.what()));
    return true;
  }

  // Add exit and entry Strategies
  ram_utils::AddEntryExitStrategies srv;
  srv.request.poses = trajectory.poses;

  if (!entry_exit_strategies_client.call(srv))
  {
    res.error = "Failed to call " + entry_exit_strategies_client.getService() + " service";
    return true;
  }

  if (!srv.response.error.empty())
  {
    res.error = srv.response.error;
    return true;
  }

  trajectory.poses = srv.response.poses;

  // Publish trajectory
  trajectory.similar_layers = false;
  trajectory.modified = ros::Time::now();
  trajectory_pub.publish(trajectory);

  return true;
}

bool rotateSelectedPosesCallback(ram_modify_trajectory::RotateSelectedPoses::Request &req,
                                 ram_modify_trajectory::RotateSelectedPoses::Response &res)
{
  std::lock_guard<std::mutex> lock(layers_mutex);
  if (req.poses.empty()) // Request is empty
  {
    res.error = "Request is empty";
    return true;
  }

  if (layers.poses.empty())
  {
    res.error = "Trajectory is empty";
    return true;
  }

  ram_msgs::AdditiveManufacturingTrajectory trajectory(layers);
  // transform center of rotation
  Eigen::Vector3d center_affine;
  tf::pointMsgToEigen(req.center_of_rotation, center_affine);

  std::vector<ram_msgs::AdditiveManufacturingPose> selected_poses(req.poses);

  //Create transformation
  //tf: trajectory frame
  //cr: center of rotation
  // P'_tf = [T_tf_cr * R(z,angle) * T_cr_tf]*P_tf
  Eigen::Affine3d translation_tf_cr(Eigen::Affine3d::Identity());
  translation_tf_cr.translate(center_affine);

  Eigen::Affine3d rot_z(Eigen::Affine3d::Identity());
  rot_z.rotate(Eigen::AngleAxisd(req.rotation_angle, Eigen::Vector3d::UnitZ()));

  Eigen::Affine3d transformation(translation_tf_cr * rot_z * translation_tf_cr.inverse());

  //Rotate selection
  for (auto &selected_pose : req.poses)
  {
    bool pose_in_trajectory = false;

    for (auto &current_pose : trajectory.poses)
    {
      if (current_pose.unique_id.uuid != selected_pose.unique_id.uuid) // uuid are not equals
        continue;

      // uuid are equals. Update geometry pose
      pose_in_trajectory = true;
      Eigen::Affine3d current_pose_affine;
      tf::poseMsgToEigen(current_pose.pose, current_pose_affine);

      Eigen::Affine3d new_pose = Eigen::Affine3d::Identity();
      new_pose = transformation * current_pose_affine;

      geometry_msgs::Pose new_pose_msg;
      tf::poseEigenToMsg(new_pose, new_pose_msg);
      current_pose.pose = new_pose_msg; // Update current ram pose

      break;
    }
    if (!pose_in_trajectory)
    {
      res.error = "Pose in the request that are not in the trajectory";
      return true;
    }
  }

  trajectory.modified = ros::Time::now();
  trajectory_pub.publish(trajectory);

  return true;

}

bool reflectSelectedPosesCallback(ram_modify_trajectory::ReflectSelectedPoses::Request &req,
                                  ram_modify_trajectory::ReflectSelectedPoses::Response &res)
{
  std::lock_guard<std::mutex> lock(layers_mutex);
  ram_msgs::AdditiveManufacturingTrajectory trajectory(layers);

  if (req.poses.empty())
  {
    res.error = "Request pose selection is empty";
    return true;
  }

  if (trajectory.poses.empty())
  {
    res.error = "Trajectory is empty";
    return true;
  }

  // Transform center of rotation
  Eigen::Vector3d p_plane;
  tf::pointMsgToEigen(req.point_on_plane, p_plane);

  Eigen::Vector3d n_plane;
  tf::vectorMsgToEigen(req.normal_vector, n_plane);
  n_plane.normalize();

  // Create transformation
  // o: origin
  Eigen::Affine3d trans_o_plane(Eigen::Affine3d::Identity());
  trans_o_plane.translate(p_plane);

  // Householder transformation (Using to reflection across a plane)
  Eigen::Affine3d householder(Eigen::Affine3d::Identity());
  householder.matrix().topLeftCorner(3, 3) = Eigen::Matrix3d::Identity() - 2 * n_plane * n_plane.transpose();

  Eigen::Affine3d reflect(trans_o_plane * householder * trans_o_plane.inverse());

  // Reflect selection
  for (auto &selected_pose : req.poses)
  {
    bool pose_in_trajectory = false;

    for (auto &current_pose : trajectory.poses)
    {
      if (current_pose.unique_id.uuid != selected_pose.unique_id.uuid) // uuid are not equals
        continue;

      // uuid are equals. Update geometry pose
      pose_in_trajectory = true;

      Eigen::Affine3d current_pose_affine;
      tf::poseMsgToEigen(current_pose.pose, current_pose_affine);

      //  Reflect the pose position
      Eigen::Affine3d new_pose = current_pose_affine;
      new_pose.translation() = reflect * current_pose_affine.translation();

      geometry_msgs::Pose new_pose_msg;
      tf::poseEigenToMsg(new_pose, new_pose_msg);
      current_pose.pose = new_pose_msg; // Update current ram pose

      break;
    }
    if (!pose_in_trajectory) // Pose in the request that are not in the trajectory
    {
      res.error = "Some poses in the request are not in the trajectory";
      return true;
    }
  }

  trajectory.modified = ros::Time::now();
  trajectory_pub.publish(trajectory);

  return true;
}

bool scaleSelectedPosesCallback(ram_modify_trajectory::ScaleSelectedPoses::Request &req,
                                ram_modify_trajectory::ScaleSelectedPoses::Response &res)
{
  std::lock_guard<std::mutex> lock(layers_mutex);
  ram_msgs::AdditiveManufacturingTrajectory trajectory(layers);

  if (req.poses.empty())
  {
    res.error = "Request pose selection is empty";
    return true;
  }

  if (trajectory.poses.empty())
  {
    res.error = "Trajectory is empty";
    return true;
  }

  // Transform center of rotation
  Eigen::Vector3d center_of_scaling;
  tf::pointMsgToEigen(req.center_of_scaling, center_of_scaling);

  // Create transformation
  // o: origin
  Eigen::Affine3d translation(Eigen::Affine3d::Identity());
  translation.translate(center_of_scaling);

  Eigen::Affine3d scaling(Eigen::Affine3d::Identity());
  scaling.scale(Eigen::Vector3d(req.scale_factor, req.scale_factor, 1));

  Eigen::Affine3d transformation(translation * scaling * translation.inverse());
  // Scale selection
  for (auto &selected_pose : req.poses)
  {
    bool pose_in_trajectory = false;

    for (auto &current_pose : trajectory.poses)
    {
      if (current_pose.unique_id.uuid != selected_pose.unique_id.uuid) // uuid are not equals
        continue;

      // uuid are equals. Update geometry pose
      pose_in_trajectory = true;

      Eigen::Affine3d current_pose_affine;
      tf::poseMsgToEigen(current_pose.pose, current_pose_affine);

      // Scale the pose position
      Eigen::Affine3d new_pose = current_pose_affine;
      new_pose.translation() = transformation * current_pose_affine.translation();

      geometry_msgs::Pose new_pose_msg;
      tf::poseEigenToMsg(new_pose, new_pose_msg);
      current_pose.pose = new_pose_msg; // Update current ram pose

      break;
    }
    if (!pose_in_trajectory) // Pose in the request that are not in the trajectory
    {
      res.error = "Some poses in the request are not in the trajectory";
      return true;
    }
  }

  trajectory.modified = ros::Time::now();
  trajectory_pub.publish(trajectory);

  return true;
}

bool shiftPosesCallback(ram_modify_trajectory::ShiftPoses::Request &req,
                        ram_modify_trajectory::ShiftPoses::Response &res)
{
  std::lock_guard<std::mutex> lock(layers_mutex);
  ram_msgs::AdditiveManufacturingTrajectory trajectory(layers);

  if (req.poses.empty())
  {
    res.error = "Request pose selection is empty";
    return true;
  }

  if (trajectory.poses.empty())
  {
    res.error = "Trajectory is empty";
    return true;
  }

  for (auto &selected_pose : req.poses)
  {
    bool pose_in_trajectory = false;

    for (auto &current_pose : trajectory.poses)
    {
      if (current_pose.unique_id.uuid != selected_pose.unique_id.uuid) // uuid are not equals
        continue;

      pose_in_trajectory = true;

      //  Modify pose
      // How much to offset
      double offset = current_pose.pose.position.z * tan(req.angle_z);

      // Compute real offset given the offset direction
      current_pose.pose.position.x += offset * cos(req.direction_angle);
      current_pose.pose.position.y += offset * sin(req.direction_angle);
      break;
    }

    if (!pose_in_trajectory) // Pose in the request that are not in the trajectory
    {
      res.error = "Some poses in the request are not in the trajectory";
      return true;
    }
  }

  trajectory.modified = ros::Time::now();
  trajectory_pub.publish(trajectory);
  return true;
}

bool trajectoryInterruptionCallback(ram_modify_trajectory::TrajectoryInterruption::Request &req,
                                    ram_modify_trajectory::TrajectoryInterruption::Response &res)
{
  std::lock_guard<std::mutex> lock(layers_mutex);
  ram_msgs::AdditiveManufacturingTrajectory trajectory(layers);

  if (req.poses.empty())
  {
    res.error = "Request pose selection is empty";
    return true;
  }

  if (trajectory.poses.empty())
  {
    res.error = "Trajectory is empty";
    return true;
  }

  for (auto &selected_pose: req.poses)
  {
    if (selected_pose.polygon_start)
    {
      res.error = "Cannot select polygon_start poses";
      return true;
    }
    else if (selected_pose.polygon_end)
    {
      res.error = "Cannot select polygon_end poses";
      return true;
    }
    else if (selected_pose.entry_pose)
    {
      res.error = "Cannot select entry_pose poses";
      return true;
    }
    else if (selected_pose.exit_pose)
    {
      res.error = "Cannot select exit_pose poses";
      return true;
    }
  }

  for (auto &selected_pose : req.poses)
  {
    bool pose_in_trajectory = false;

    for (auto it(trajectory.poses.begin()); it != trajectory.poses.end(); ++it)
    {
      if (it->unique_id.uuid != selected_pose.unique_id.uuid) // uuid are not equals
        continue;

      pose_in_trajectory = true;

      if (req.add_pose)
      {
        it->polygon_end = false;
        it->polygon_start = true;

        ram_msgs::AdditiveManufacturingPose new_pose(*it);
        new_pose.unique_id = unique_id::toMsg(unique_id::fromRandom());
        new_pose.polygon_start = false;
        new_pose.polygon_end = true;
        trajectory.poses.insert(it, new_pose);
      }
      else
      {
        if ((it + 1) == trajectory.poses.end())
        {
          res.error = "Cannot add interruption at the end of the trajectory";
          return true;
        }

        it->polygon_start = false;
        it->polygon_end = true;

        ++it;
        it->polygon_start = true;
        it->polygon_end = false;
      }
      break;
    }

    if (!pose_in_trajectory)
    {
      res.error = "Pose is not in the trajectory";
      return true;
    }
  }

  // Add exit and entry Strategies
  ram_utils::AddEntryExitStrategies srv;
  srv.request.poses = trajectory.poses;

  if (!entry_exit_strategies_client.call(srv))
  {
    res.error = "Failed to call " + entry_exit_strategies_client.getService() + " service";
    return true;
  }

  if (!srv.response.error.empty())
  {
    res.error = srv.response.error;
    return true;
  }

  trajectory.poses = srv.response.poses;

  trajectory.modified = ros::Time::now();
  trajectory_pub.publish(trajectory);
  return true;
}

using LayerIndexZPosition = std::map<uint64_t, double>;

void computeNewPosition(const ram_msgs::AdditiveManufacturingTrajectory &trajectory,
                        const ram_msgs::AdditiveManufacturingTrajectoryInfo &info,
                        ram_modify_trajectory::ChangeLayerHeightRequest &req,
                        std::vector<double> &entry_exit_z_position,
                        LayerIndexZPosition &new_z_positions)
{
  std::set<uint64_t> selected_layers;
  for (auto &pose : req.poses) // list of layers to be modified
  {
    if (pose.layer_index == info.number_of_layers_levels - 1)
      continue; // Ignore last layer

    selected_layers.insert(pose.layer_index + 1);
  }

  // First list all layer indexes and z position within this trajectory
  // without taking into account entry and exit poses
  // map data structure = {Layer index, Layer position}
  LayerIndexZPosition current_z_positions;
  for (auto &current_pose : trajectory.poses)
  {
    if (!(current_pose.entry_pose || current_pose.exit_pose))
      current_z_positions.insert(std::pair<uint64_t, double>(current_pose.layer_index, current_pose.pose.position.z));
    else
      continue;
  }

  new_z_positions.clear();
  // Map data structure = {Layer_index, Layer_position}
  // New trajectory generation
  for (auto &current_pose : trajectory.poses)
  {
    if (current_pose.layer_index == 0)
    {
      new_z_positions.insert(std::pair<uint64_t, double>(0, 0.0));
      continue;
    }
    auto prev_z_new_layer = new_z_positions.find(current_pose.layer_index - 1);
    auto prev_z_layer = current_z_positions.find(current_pose.layer_index - 1);
    auto current_z_layer = current_z_positions.find(current_pose.layer_index);

    double new_position = prev_z_new_layer->second + std::fabs(current_z_layer->second - prev_z_layer->second);

    if (selected_layers.find(current_pose.layer_index) == selected_layers.end() || selected_layers.empty())
      new_z_positions.insert(std::pair<uint64_t, double>(current_pose.layer_index, new_position));
    else
    {
      if (req.absolute_mode)
      {
        new_position = prev_z_new_layer->second + req.height;
        new_z_positions.insert(std::pair<uint64_t, double>(current_pose.layer_index, new_position));
      }
      else
      {
        if (std::fabs(current_z_layer->second - prev_z_layer->second) + req.height <= 0)
          throw std::runtime_error("The layers would be inverted if the requested height is applied");
        else
          new_position =
              prev_z_new_layer->second + std::fabs(current_z_layer->second - prev_z_layer->second) + req.height;
        new_z_positions.insert(std::pair<uint64_t, double>(current_pose.layer_index, new_position));
      }
    }
  }

  // Entry and exit poses management
  entry_exit_z_position.clear();
  for (auto &current_pose : trajectory.poses)
  {
    if (current_pose.entry_pose)
      entry_exit_z_position.emplace_back(current_pose.pose.position.z);

    else if (current_pose.exit_pose)
    {
      auto prev_layer_position_current_traj = current_z_positions.find(current_pose.layer_index);
      auto prev = new_z_positions.find(current_pose.layer_index);

      double exit_pose_height = std::fabs(current_pose.pose.position.z - prev_layer_position_current_traj->second);
      entry_exit_z_position.emplace_back(prev->second + exit_pose_height);
    }
    else
      continue;
  }
}

bool changeLayerHeightCallback(ram_modify_trajectory::ChangeLayerHeightRequest &req,
                               ram_modify_trajectory::ChangeLayerHeightResponse &res)
{
  std::lock_guard<std::mutex> lock(layers_mutex);
  std::lock_guard<std::mutex> lock_info(info_mutex);
  ram_msgs::AdditiveManufacturingTrajectory trajectory(layers);
  ram_msgs::AdditiveManufacturingTrajectoryInfo info(info_trajectory);

  if (trajectory.poses.empty())
  {
    res.error = "Trajectory is empty";
    return true;
  }

  if (req.poses.empty())
  {
    res.error = "Request pose selection is empty";
    return true;
  }

  if (info.number_of_polygons > 1)
  {
    res.error = "Impossible to modify: the shape must be made only of one polygon";
    return true;
  }

  if (req.absolute_mode && req.height <= 0)
  {
    res.error = "Impossible to modify: height must be strictly positive in absolute mode";
    return true;
  }

  std::vector<double> entry_exit_z_position;
  LayerIndexZPosition computed_new_position;
  try
  {
    computeNewPosition(trajectory, info, req, entry_exit_z_position, computed_new_position);
  }
  catch (const std::runtime_error &e)
  {
    res.error = std::string(e.what());
    return true;
  }

  for (auto &data : computed_new_position)
  {
    for (auto &current_pose : trajectory.poses)
    {
      if (current_pose.entry_pose)
      {
        current_pose.pose.position.z = entry_exit_z_position.front();
        continue;
      }

      if (current_pose.exit_pose)
      {
        current_pose.pose.position.z = entry_exit_z_position.back();
        continue;
      }

      // Check if its index has to be modified i.e. if the current index is in the layers_to_be_modified
      try
      {
        if (data.first == current_pose.layer_index)
          current_pose.pose.position.z = data.second;
        else
          continue;
      }
      catch (const std::runtime_error &e)
      {
        res.error = "Error during new trajectory generation\n"
                    "Some poses in the request are not in the trajectory\n"
                    "Error : " + std::string(e.what());
        return true;
      }
    }
  }

  trajectory.modified = ros::Time::now();
  trajectory_pub.publish(trajectory);
  return true;
}

bool computeAngleBetweenThreePoints(double &res,
                                    const geometry_msgs::Point &point_a,
                                    const geometry_msgs::Point &point_b,
                                    const geometry_msgs::Point &point_c)
{
  // In case of superposed points within the trajectory
  if (((point_a.x == point_b.x) && (point_a.y == point_b.y)) || ((point_b.x == point_c.x) && (point_b.y == point_c.y)))
    return false;

  double dot_product =
      (point_b.x - point_a.x) * (point_c.x - point_b.x) + (point_b.y - point_a.y) * (point_c.y - point_b.y);
  double determinant =
      (point_b.x - point_a.x) * (point_c.y - point_b.y) - (point_b.y - point_a.y) * (point_c.x - point_b.x);
  res = std::abs(atan2(determinant, dot_product));

  return true;
}

bool tweakBlendRadiusCallBack(ram_modify_trajectory::TweakBlendRadiusRequest &req,
                              ram_modify_trajectory::TweakBlendRadiusResponse &res)
{
  std::lock_guard<std::mutex> lock(layers_mutex);
  ram_msgs::AdditiveManufacturingTrajectory trajectory(layers);

  if (trajectory.poses.empty())
  {
    res.error = "Trajectory is empty";
    return true;
  }

  if (req.poses.empty())
  {
    res.error = "Request pose selection is empty";
    return true;
  }

  if (req.new_blend_radius.empty())
  {
    res.error = "The input parameters are empty";
    return true;
  }

  for (auto &row : req.new_blend_radius)
  {
    auto row_index = &row - &req.new_blend_radius.at(0);

    if (((row.max_range) - (row.min_range)) <= 0)
    {
      res.error =
          "The range of the row " + std::to_string(row_index + 1) + " is not valid because it is null !";
      return true;
    }

    if (((row.max_range) - (row.min_range)) > M_PI)
    {
      res.error =
          "The range of the row " + std::to_string(row_index + 1) + " is not valid because it exceeds 180° !";
      return true;
    }

    if ((row.max_range) < (row.min_range))
    {
      res.error = "The range of the row " + std::to_string(row_index + 1) + " is not valid because α min > α max !\n"
                                                                            "α min has to be lower than α max";
      return true;
    }

    if (row_index > 0)
    {
      double prev_alpha_max = req.new_blend_radius.at(row_index - 1).max_range;
      if (prev_alpha_max != row.min_range)
      {
        res.error = "The range of the row " + std::to_string(row_index + 1) + " is not valid because α min ≠ the " +
                    std::to_string(row_index) + " α max !";
        return true;
      }
    }
  }

  std::vector<ram_msgs::AdditiveManufacturingPose> valid_selection{};
  for (auto &selected_pose : req.poses)
  {
    // ignore entry, exit poses, polygon start and end
    if (selected_pose.entry_pose || selected_pose.exit_pose || selected_pose.polygon_start || selected_pose.polygon_end)
      continue;
    else
      valid_selection.emplace_back(selected_pose);
  }

  if (valid_selection.empty())
  {
    res.error = "The selected poses are not valid.\n"
                "it's forbidden to select entry and exit pose or polygon start or end";
    return true;
  }

  for (auto &selected_pose : valid_selection)
  {
    auto pose = std::find_if(trajectory.poses.begin(), trajectory.poses.end(), [=](const ram_msgs::AdditiveManufacturingPose &p)
    {
      return !(selected_pose.unique_id.uuid != p.unique_id.uuid);
    });

    if (pose == trajectory.poses.end())
    {
      res.error = "The selected pose does not exist within the trajectory";
      return true;
    }

    const std::size_t trajectory_index(std::distance(trajectory.poses.begin(), pose));

    geometry_msgs::Point point_prev{};
    if (trajectory_index > 0)
      // find the previous point within the trajectory
      point_prev = trajectory.poses.at(trajectory_index - 1).pose.position;
    else
      continue;

    // find the next point within the trajectory
    geometry_msgs::Point point_next = trajectory.poses.at(trajectory_index + 1).pose.position;

    double angle(0);
    if (!computeAngleBetweenThreePoints(angle, point_prev, selected_pose.pose.position, point_next))
    {
      res.error = "The selected pose " + std::to_string(trajectory_index) +
                  " is not valid because one of its neighbours is superposed with it";
      return true;
    }

    auto line = std::find_if(req.new_blend_radius.begin(), req.new_blend_radius.end(),
        [=](const ram_modify_trajectory::BlendRadiusModification &line)
        {
          return (line.min_range <= angle && angle < line.max_range);
        });

    if (line == req.new_blend_radius.end())
      continue; // <=> no modification on this pose

    if (line->absolute)
      pose->params.blend_radius = line->blend_radius_value;
    else
    {
      pose->params.blend_radius += line->blend_radius_value;
      if (pose->params.blend_radius > 100.0)
        pose->params.blend_radius = 100.0;
      else if (pose->params.blend_radius <= 0.0)
        pose->params.blend_radius = 0.0;
    }
  }

  trajectory.modified = ros::Time::now();
  trajectory_pub.publish(trajectory);
  return true;
}

bool simplifyTrajectoryCallBack(ram_modify_trajectory::SimplifyTrajectoryRequest &req,
                                ram_modify_trajectory::SimplifyTrajectoryResponse &res)
{
  std::lock_guard<std::mutex> lock(layers_mutex);

  // Gather whole trajectory and work on it
  ram_msgs::AdditiveManufacturingTrajectory trajectory(layers), trajectory_copy(layers);

  // Check input parameters
  if (trajectory_copy.poses.empty())
  {
    res.error = "Trajectory to modify is empty";
    return true;
  }

  if (req.poses.empty())
  {
    res.error = "Request pose selection is empty";
    return true;
  }

  if (req.area_threshold <= 0)
  {
    res.error = "The input parameter is not valid, area_threshold is negative or null";
    return true;
  }

  bool trajectory_modified(false);
  // Modify poses
  for (auto current_sel_pose_it(req.poses.begin()); current_sel_pose_it != req.poses.end(); ++current_sel_pose_it)
  {
    for (auto it(trajectory_copy.poses.begin()); it != trajectory_copy.poses.end(); ++it)
    {
      // If the current iterator(it) pointes the 3rd pose before the end of the trajectory,
      // we are not able to compute the it + 2, so we break the loop
      if (std::distance(it, trajectory_copy.poses.end()) <= 3)
        break;

      // Ignore entry_pose, exit_pose and polygon_end as input poses
      if(current_sel_pose_it->entry_pose || current_sel_pose_it->exit_pose || current_sel_pose_it->polygon_end)
        continue;

      // uuid are not equals
      if (current_sel_pose_it->unique_id.uuid != it->unique_id.uuid)
        continue;

    /* Method applied in order to compute the triangles area :
     * for a given triangle ABC :
     *
     * (xa, ya) A x----x B (xb, yb)
     *             \  |
     *              \|
     *               x C (xc, yc)
     *
     *  Its surface can be determined by its coordinates, by using the determinant calculation
     *  A = abs(0.5 x det |xa xb xc| ), with  |xa xb xc| the triangle matrix
     *                    |ya yb yc|          |ya yb yc|
     *                    |1  1  1 |          |1  1  1 |
     *  in fact each colomn is equal to its coordinates vectors.
     */
      Eigen::Matrix3d triangle_matrix;
      triangle_matrix.col(0) << (it)->pose.position.x, (it)->pose.position.y, 1.0;
      triangle_matrix.col(1) << (it + 1)->pose.position.x, (it + 1)->pose.position.y, 1.0;
      triangle_matrix.col(2) << (it + 2)->pose.position.x, (it + 2)->pose.position.y, 1.0;

      // Compute the determinant and the surface and find the associated pose into the input trajectory in order to delete it
      if((it + 1)->entry_pose || (it + 1)->exit_pose || (it + 1)->polygon_start || (it + 1)->polygon_end)
          continue;
      else if(std::abs(triangle_matrix.determinant() / 2.0) < req.area_threshold)
      {
        std::vector<ram_msgs::AdditiveManufacturingPose>::iterator tmp_it(trajectory.poses.begin());
        for (; tmp_it != trajectory.poses.end(); ++tmp_it)
        {
          if (tmp_it->pose.position.x == (it + 1)->pose.position.x &&
              (tmp_it->pose.position.y == (it + 1)->pose.position.y))
            break;
        }
        trajectory_modified = true;
        trajectory.poses.erase(tmp_it);
      }
    }
  }

  trajectory.modified = ros::Time::now();
  trajectory_pub.publish(trajectory);

  if (!trajectory_modified)
    res.error = "The current selection has not been modified, please change the selection";
  return true;
}

bool pushPullAngleCallback(ram_modify_trajectory::PushPullAngleRequest &req,
                           ram_modify_trajectory::PushPullAngleResponse &res)
{
  ram_msgs::AdditiveManufacturingTrajectory trajectory;
  {
    std::lock_guard<std::mutex> lock(layers_mutex);
    trajectory = layers;
  }

  if (req.poses.empty())
  {
    res.error = "Request is empty";
    return true;
  }

  if (trajectory.poses.empty())
  {
    res.error = "Trajectory is empty";
    return true;
  }

  if (req.angle == 0)
  {
    res.error = "Angle cannot be zero";
    return true;
  }

  using PoseIt = std::vector<ram_msgs::AdditiveManufacturingPose>::iterator;
  // Ignore last pose (there is not next pose to compute the direction)

  bool printing(false);
  for (PoseIt it(trajectory.poses.begin()); it != --trajectory.poses.end(); ++it)
  {
    if (it->polygon_start)
      printing = true;
    else if (it->polygon_end)
      printing = false;

    // Skip poses that are not in the request (don't modify them)
    bool modify_this_pose(false);
    for (auto p: req.poses)
    {
      if (p.unique_id == it->unique_id)
      {
        modify_this_pose = true;
        break;
      }
    }

    if (!modify_this_pose)
      continue;

    if (!printing && !it->polygon_end) // We want to modify polygon_end poses
      continue;

    PoseIt ref_pose(it + 1);
     // If current pose is polygon end, use previous pose as a reference to compute direction
    if (it->polygon_end)
      ref_pose -= 2;

    Eigen::Isometry3d current, ref;
    tf::poseMsgToEigen(it->pose, current);
    tf::poseMsgToEigen(ref_pose->pose, ref);

    Eigen::Vector3d direction(ref.translation() - current.translation());
    if (it->polygon_end)
      direction = -direction;

    direction.z() = 0; // Always work in the XY plane. This algorithm does not work for 3D trajectories
    Eigen::Vector3d z(0, 0, 1); // Algorithm works only if layers are layed along the Z axis
    Eigen::Vector3d y(z.cross(direction));
    y.normalize(); // Always normalize a vector used in a angle axis

    // Rotate the pose along the Y axis
    Eigen::AngleAxisd rot(req.angle, y);
    current.linear() = rot * current.linear();

    // Update pose in the trajectory
    tf::poseEigenToMsg(current, it->pose);
  }

  trajectory.modified = ros::Time::now();
  trajectory_pub.publish(trajectory);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "modify_trajectory");
  ros::NodeHandle nh;

  // Publish on "ram/trajectory"
  trajectory_pub = nh.advertise<ram_msgs::AdditiveManufacturingTrajectory>("ram/trajectory", 10, true);

  // Get unmodified trajectory from buffer node
  unmodified_trajectory_client = nh.serviceClient<ram_utils::UnmodifiedTrajectory>(
      "ram/buffer/get_unmodified_trajectory");
  // Service to add strategies
  entry_exit_strategies_client = nh.serviceClient<ram_utils::AddEntryExitStrategies>(
      "ram/information/add_entry_exit_strategies");

  // Subscribe on trajectory and information
  ros::Subscriber sub = nh.subscribe("ram/trajectory", 10, trajectoryCallback);
  ros::Subscriber info_sub = nh.subscribe("ram/information/trajectory", 10, trajectoryInfoCallback);

  // Services to modify trajectory
  ros::ServiceServer service_1 = nh.advertiseService("ram/modify_trajectory/modify_selected_poses",
                                                     modifySelectedPosesCallback);
  ros::ServiceServer service_2 = nh.advertiseService("ram/modify_trajectory/reset_selected_poses",
                                                     resetSelectedPosesCallback);
  ros::ServiceServer service_3 = nh.advertiseService("ram/modify_trajectory/add_poses", addPosesCallback);
  ros::ServiceServer service_4 = nh.advertiseService("ram/modify_trajectory/rotate_selected_poses",
                                                     rotateSelectedPosesCallback);
  ros::ServiceServer service_5 = nh.advertiseService("ram/modify_trajectory/reflect_selected_poses",
                                                     reflectSelectedPosesCallback);
  ros::ServiceServer service_6 = nh.advertiseService("ram/modify_trajectory/delete_selected_poses",
                                                     deleteSelectedPosesCallback);
  ros::ServiceServer service_7 = nh.advertiseService("ram/modify_trajectory/scale_selected_poses",
                                                     scaleSelectedPosesCallback);
  ros::ServiceServer service_8 = nh.advertiseService("ram/modify_trajectory/shift_poses",
                                                     shiftPosesCallback);
  ros::ServiceServer service_9 = nh.advertiseService("ram/modify_trajectory/trajectory_interruption",
                                                     trajectoryInterruptionCallback);
  ros::ServiceServer service_10 = nh.advertiseService("ram/modify_trajectory/change_layer_height",
                                                      changeLayerHeightCallback);
  ros::ServiceServer service_11 = nh.advertiseService("ram/modify_trajectory/tweak_blend_radius",
                                                      tweakBlendRadiusCallBack);
  ros::ServiceServer service_12 = nh.advertiseService("ram/modify_trajectory/simplify_trajectory",
                                                      simplifyTrajectoryCallBack);
  ros::ServiceServer service_13 = nh.advertiseService("ram/modify_trajectory/push_pull_angle",
                                                      pushPullAngleCallback);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
