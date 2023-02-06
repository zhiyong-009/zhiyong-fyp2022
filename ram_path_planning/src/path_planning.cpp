#include <actionlib/server/action_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_path_planning/ContoursAction.h>
#include <ram_path_planning/contours.hpp>
#include <ram_path_planning/DonghongDingAction.h>
#include <ram_path_planning/donghong_ding.hpp>
#include <ram_path_planning/FollowPosesAction.h>
#include <ram_path_planning/follow_poses.hpp>
#include <ram_path_planning/mesh_slicer.hpp>
#include <ram_path_planning/PolygonOffsetsAction.h>
#include <ram_path_planning/polygon_offsets.hpp>
#include <ram_path_planning/ProfileAction.h>
#include <ram_path_planning/profile.hpp>
#include <ram_path_planning/RevolveAction.h>
#include <ram_path_planning/revolve.hpp>
#include <ram_path_planning/vtkRenderUpdaterTimer.hpp>
#include <ram_utils/file_extension.hpp>
#include <ros/ros.h>
#include <string>
#include <strings.h>
#include <visualization_msgs/Marker.h>

using Polygon = vtkSmartPointer<vtkPolyData>;
using PolygonVector = std::vector<Polygon>;
using Layer = std::vector<PolygonVector>;

bool use_gui;

std::unique_ptr<ros::Publisher> traj_pub;

std::unique_ptr<ram_path_planning::DonghongDing<ram_path_planning::DonghongDingAction>> donghongding_generator;
std::unique_ptr<ram_path_planning::Contours<ram_path_planning::ContoursAction>> contour_generator;
std::unique_ptr<ram_path_planning::FollowPoses<ram_path_planning::FollowPosesAction>> follow_poses_generator;
std::unique_ptr<ram_path_planning::Revolve<ram_path_planning::RevolveAction>> revolve_generator;
std::unique_ptr<ram_path_planning::PolygonOffsets<ram_path_planning::PolygonOffsetsAction>> polygon_offsets_generator;
std::unique_ptr<ram_path_planning::Profile<ram_path_planning::ProfileAction>> profile_generator;

//allow to visualize the generation of trajectory
vtkSmartPointer<vtkRendererUpdaterTimer> cb = vtkSmartPointer<vtkRendererUpdaterTimer>::New();

// Donghong Ding algorithm
void donghongDingAlgorithmGoalCb(actionlib::ServerGoalHandle<ram_path_planning::DonghongDingAction> gh)
{
  // Change status to ACTIVE
  gh.setAccepted();

  // Action elements
  ram_path_planning::DonghongDingGoal goal;
  ram_path_planning::DonghongDingResult result;
  goal = *gh.getGoal();

  // Verify parameters
  if (goal.file.empty())
  {
    result.error_msg = "File name cannot be empty";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.height_between_layers <= 0)
  {
    result.error_msg = "Height between layers cannot be <= 0";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.deposited_material_width <= 0)
  {
    result.error_msg = "Deposited material width cannot be <= 0";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }
  if (goal.contours_filtering_tolerance < 0)
  {
    result.error_msg = "Contours filtering tolerance cannot be < 0";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  // Verify file extension
  bool is_yaml_file(false);
  bool is_svg_file(false);
  const std::string file_extension(ram_utils::fileExtension(goal.file));
  if (!strcasecmp(file_extension.c_str(), "svg"))
  {
    is_svg_file = true;

    result.error_msg = "SVG files are not supported.";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }
  else if (!strcasecmp(file_extension.c_str(), "yaml") || !strcasecmp(file_extension.c_str(), "yml"))
  {
    is_yaml_file = true;
    if (goal.number_of_layers < 1)
    {
      result.error_msg = "Number of layers cannot be < 1";
      ROS_ERROR_STREAM(result.error_msg);
      if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        gh.setAborted(result);
      return;
    }
  }
  else if (strcasecmp(file_extension.c_str(), "obj") && strcasecmp(file_extension.c_str(), "ply")
           && strcasecmp(file_extension.c_str(), "stl"))
  {
    result.error_msg = "File is not a SVG, YAML, YML, OBJ, PLY or STL file: " + goal.file;
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  // Create Trajectory message
  ram_msgs::AdditiveManufacturingTrajectory msg;
  msg.file = goal.file;

  // Add request parameter to the trajectory
  msg.generation_info = "Generation algorithm = Donghong Ding\n";

  std::stringstream value;
  value << std::fixed << std::setprecision(3) << goal.height_between_layers * 1000.0;
  std::string s(value.str());

  msg.generation_info += "Layer height = " + s + " mm\n";
  s.clear();
  value.str("");

  value << std::fixed << std::setprecision(3) << goal.deposited_material_width * 1000.0;
  s = value.str();
  msg.generation_info += "Deposited material width = " + s + " mm\n";
  s.clear();
  value.str("");

  value << std::fixed << std::setprecision(3) << goal.number_of_layers * goal.height_between_layers * 1000.0;
  s = value.str();
  msg.generation_info += "Total height = " + s + " mm\n";
  s.clear();
  value.str("");

  if (!is_yaml_file && !is_svg_file)
  {
    msg.generation_info += "Slicing direction = " + std::to_string(goal.slicing_direction.x) + ", "
                           + std::to_string(goal.slicing_direction.y) + ", " + std::to_string(goal.slicing_direction.z)
                           + "\n";
  }
  else
  {
    msg.generation_info += "Number of layers = " + std::to_string(goal.number_of_layers) + "\n";
  }

  value << std::fixed << std::setprecision(3) << goal.contours_filtering_tolerance * 1000.0;
  s = value.str();
  msg.generation_info += "Contours filtering tolerance = " + s + " mm\n";

  msg.similar_layers = (is_yaml_file || is_svg_file);

  std::array<double, 3> slicing_direction;
  slicing_direction[0] = goal.slicing_direction.x;
  slicing_direction[1] = goal.slicing_direction.y;
  slicing_direction[2] = goal.slicing_direction.z;

  std::vector<Layer> additive_traj;
  cb->current_layer_.clear();

  // Generate trajectory
  // Action feedback
  if (!donghongding_generator->publishStatusPercentageDone("Generating trajectory", 10, gh))
    return;

  if (is_svg_file || is_yaml_file)
  {
    std::string error_message;
    error_message = donghongding_generator->generateOneLayerTrajectory(gh, 10, 50, goal.file, cb->current_layer_,
                                                                       goal.deposited_material_width,
                                                                       goal.contours_filtering_tolerance, M_PI / 6,
                                                                       false,
                                                                       use_gui);

    if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE)
      return;

    if (error_message.empty())
    {
      // Action feedback
      if (!donghongding_generator->publishStatusPercentageDone(
          "Trajectory has been generated for one layer.\n Duplicating and connecting layers", 50, gh))
        return;

      donghongding_generator->connectLayers(gh, 50, 90, cb->current_layer_, msg, goal.number_of_layers,
                                            goal.height_between_layers);

    }
    else
    {
      result.error_msg = error_message;
      ROS_ERROR_STREAM(result.error_msg);
      if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        gh.setAborted(result);
      return;
    }
  }
  else
  {
    // Slice the mesh
    const unsigned nb_layers(
        ram_path_planning::sliceMesh(additive_traj, goal.file, cb->mesh_, cb->stripper_, goal.height_between_layers,
                                     slicing_direction,
                                     use_gui));

    if (nb_layers < 1)
    {
      result.error_msg = "Slicing failed, zero slice generated";
      ROS_ERROR_STREAM(result.error_msg);
      if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        gh.setAborted(result);
      return;
    }
    // Action feedback
    if (!donghongding_generator->publishStatusPercentageDone("Slicing completed", 20, gh))
      return;

    // Generate trajectory on each layer
    for (unsigned i(0); i < additive_traj.size(); ++i)
    {
      Polygon contours(additive_traj[i][0][0]);
      contours->DeepCopy(additive_traj[i][0][0]);

      int current_progress_value = 20 + i * 30 / nb_layers; // first value is 20%
      int next_progress_value = 20 + (i + 1) * 30 / nb_layers; // last value is 50%

      std::string error_message;
      error_message = donghongding_generator->generateOneLayerTrajectory(gh, current_progress_value,
                                                                         next_progress_value,
                                                                         contours,
                                                                         cb->current_layer_,
                                                                         goal.deposited_material_width,
                                                                         goal.contours_filtering_tolerance,
                                                                         slicing_direction,
                                                                         M_PI / 6,
                                                                         false,
                                                                         use_gui);

      if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE)
        return;

      if (error_message.empty())
      {
        // Action feedback
        if (!donghongding_generator->publishStatusPercentageDone(
            "Trajectory of layer " + std::to_string(i) + " has been generated", next_progress_value,
            gh)) // percentage between 20% and 50%
          return;
      }
      else
      {
        result.error_msg = error_message + "\n" + "Could not generate layer " + std::to_string(i);
        ROS_ERROR_STREAM(result.error_msg);
        if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
          gh.setAborted(result);
        return;
      }
      additive_traj[i] = cb->current_layer_;
    }

    std::string return_message;
    return_message = donghongding_generator->connectMeshLayers(gh, 50, 90, additive_traj, msg);

    if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE)
      return;

    if (!return_message.empty())
    {
      result.error_msg = return_message + "\n" + "Error connecting layers";
      ROS_ERROR_STREAM(result.error_msg);
      if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        gh.setAborted(result);
      return;
    }
  }

  // Trajectory is now complete
  // Create a message
  // Fill response and publish trajectory

  // Action feedback
  if (!donghongding_generator->publishStatusPercentageDone(
      "Layers have been connected\nCreating message to publish trajectory", 90, gh))

    if (msg.poses.size() == 0)
    {
      result.error_msg = "Trajectory is empty";
      ROS_ERROR_STREAM(result.error_msg);
      if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        gh.setAborted(result);
      return;
    }
  result.number_of_poses = msg.poses.size();

  // Set the time to zero, the trajectory is incomplete at this stage
  // It will be published on trajectory_tmp, completed by fill_trajectory and then published on trajectory
  msg.generated.nsec = 0;
  msg.generated.sec = 0;
  msg.modified = msg.generated;

  // Add UUIDs
  for (auto &pose : msg.poses)
    pose.unique_id = unique_id::toMsg(unique_id::fromRandom());

  if (!donghongding_generator->publishStatusPercentageDone("Trajectory has been generated and published", 100, gh))
    return;

  if (traj_pub->getNumSubscribers() == 0)
    ROS_WARN_STREAM("No subscriber on topic " << traj_pub->getTopic());

  traj_pub->publish(msg);

  if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE
      && gh.getGoalStatus().status != actionlib_msgs::GoalStatus::PREEMPTING)
    return;

  gh.setSucceeded(result);
}

void donghongDingAlgorithmCancelCb(actionlib::ServerGoalHandle<ram_path_planning::DonghongDingAction> gh)
{
  if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE
      || gh.getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTING)
    gh.setCanceled();
}

// Contours algorithm
void contoursAlgorithmGoalCb(actionlib::ServerGoalHandle<ram_path_planning::ContoursAction> gh)
{
  // Change status to ACTIVE
  gh.setAccepted();

  // Action elements
  ram_path_planning::ContoursGoal goal;
  ram_path_planning::ContoursResult result;
  goal = *gh.getGoal();

  // Verify parameters
  if (goal.file.empty())
  {
    result.error_msg = "File name cannot be empty";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.height_between_layers <= 0)
  {
    result.error_msg = "Height between layers cannot be <= 0";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.deposited_material_width <= 0)
  {
    result.error_msg = "Deposited material width cannot be <= 0";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  // Verify file extension
  bool is_yaml_file(false);
  const std::string file_extension(ram_utils::fileExtension(goal.file));
  if (!strcasecmp(file_extension.c_str(), "svg"))
  {
    result.error_msg = "SVG files are not supported.";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }
  else if (!strcasecmp(file_extension.c_str(), "yaml") || !strcasecmp(file_extension.c_str(), "yml"))
  {
    is_yaml_file = true;
    if (goal.number_of_layers < 1)
    {
      result.error_msg = "Number of layers cannot be < 1";
      ROS_ERROR_STREAM(result.error_msg);
      if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        gh.setAborted(result);
      return;
    }
  }
  else if (strcasecmp(file_extension.c_str(), "obj") && strcasecmp(file_extension.c_str(), "ply")
           && strcasecmp(file_extension.c_str(), "stl"))
  {
    result.error_msg = "File is not a YAML, YML, OBJ, PLY or STL file: " + goal.file;
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  // Create Trajectory message
  ram_msgs::AdditiveManufacturingTrajectory msg;
  msg.file = goal.file;

  // Add request parameter to the trajectory
  msg.generation_info = "Generation algorithm = Contour\n";
  std::stringstream value;
  value << std::fixed << std::setprecision(3) << goal.height_between_layers * 1000.0;
  std::string s(value.str());

  msg.generation_info += "Layer height = " + s + " mm\n";
  s.clear();
  value.str("");

  value << std::fixed << std::setprecision(3) << goal.deposited_material_width * 1000.0;
  s = value.str();
  msg.generation_info += "Deposited material width = " + s + " mm\n";
  s.clear();
  value.str("");

  value << std::fixed << std::setprecision(3) << goal.number_of_layers * goal.height_between_layers * 1000.0;
  s = value.str();
  msg.generation_info += "Total height = " + s + " mm\n";

  if (!is_yaml_file)
  {
    msg.generation_info += "Slicing direction = " + std::to_string(goal.slicing_direction.x) + ", "
                           + std::to_string(goal.slicing_direction.y) + ", " + std::to_string(goal.slicing_direction.z);
  }
  else
  {
    msg.generation_info += "Number of layers = " + std::to_string(goal.number_of_layers);
  }

  msg.similar_layers = is_yaml_file;
  std::array<double, 3> slicing_direction;
  slicing_direction[0] = goal.slicing_direction.x;
  slicing_direction[1] = goal.slicing_direction.y;
  slicing_direction[2] = goal.slicing_direction.z;

  std::vector<Layer> additive_traj;
  cb->current_layer_.clear();

  // Action feedback
  if (!contour_generator->publishStatusPercentageDone("Generating trajectory", 10, gh))
    return;

  // Generate trajectory
  if (is_yaml_file)
  {
    std::string error_message;
    error_message = contour_generator->generateOneLayerTrajectory(gh, goal.file, cb->current_layer_,
                                                                  goal.deposited_material_width,
                                                                  use_gui);

    if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE)
      return;

    if (error_message.empty())
    {
      // Action feedback
      if (!contour_generator->publishStatusPercentageDone("Trajectory has been generated.\n Connecting Layers", 50, gh))
        return;

      contour_generator->connectLayers(gh, 50, 90, cb->current_layer_, msg, goal.number_of_layers,
                                       goal.height_between_layers);
    }
    else
    {
      result.error_msg = error_message;
      ROS_ERROR_STREAM(result.error_msg);
      if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        gh.setAborted(result);
      return;
    }
  }
  else
  {
    // Slice the mesh
    const unsigned nb_layers(
        ram_path_planning::sliceMesh(additive_traj, goal.file, cb->mesh_, cb->stripper_, goal.height_between_layers,
                                     slicing_direction,
                                     use_gui));

    if (nb_layers < 1)
    {
      result.error_msg = "Slicing failed, zero slice generated";
      ROS_ERROR_STREAM(result.error_msg);
      if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        gh.setAborted(result);
      return;
    }
    // Action feedback
    if (!contour_generator->publishStatusPercentageDone("Slicing completed", 20, gh))
      return;

    // Generate trajectory on each layer
    for (unsigned i(0); i < additive_traj.size(); ++i)
    {
      Polygon contours(additive_traj[i][0][0]);
      contours->DeepCopy(additive_traj[i][0][0]);

      std::string error_message;
      error_message = contour_generator->generateOneLayerTrajectory(gh, contours, cb->current_layer_,
                                                                    goal.deposited_material_width,
                                                                    slicing_direction,
                                                                    use_gui);

      if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE)
        return;

      if (error_message.empty())
      {
        // Action feedback
        if (!contour_generator->publishStatusPercentageDone(
            "Trajectory of layer " + std::to_string(i) + " has been generated", 20 + i * 30 / nb_layers,
            gh)) // percentage between 20% and 50%
          return;
      }
      else
      {
        result.error_msg = error_message + "\n" + "Could not generate layer " + std::to_string(i);
        ROS_ERROR_STREAM(result.error_msg);
        if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
          gh.setAborted(result);
        return;
      }
      additive_traj[i] = cb->current_layer_;
    }

    std::string return_message;
    return_message = contour_generator->connectMeshLayers(gh, 50, 90, additive_traj, msg);

    if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE)
      return;

    if (!return_message.empty())
    {
      result.error_msg = return_message + "\n" + "Error connecting layers";
      ROS_ERROR_STREAM(result.error_msg);
      if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        gh.setAborted(result);
      return;
    }
  }
  // Trajectory is now complete
  // Create a message
  // Fill response and publish trajectory
  // Action feedback
  if (!contour_generator->publishStatusPercentageDone("Creating message to publish trajectory", 90, gh))
    return;

  if (msg.poses.size() == 0)
  {
    result.error_msg = "Trajectory is empty";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }
  result.number_of_poses = msg.poses.size();

  // Set the time to zero, the trajectory is incomplete at this stage
  // It will be published on trajectory_tmp, completed by fill_trajectory and then published on trajectory
  msg.generated.nsec = 0;
  msg.generated.sec = 0;
  msg.modified = msg.generated;

  // Add UUID
  for (auto &pose : msg.poses)
    pose.unique_id = unique_id::toMsg(unique_id::fromRandom());

  if (!contour_generator->publishStatusPercentageDone("Trajectory has been generated and published", 100, gh))
    return;

  if (traj_pub->getNumSubscribers() == 0)
    ROS_WARN_STREAM("No subscriber on topic " << traj_pub->getTopic());

  traj_pub->publish(msg);

  if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE
      && gh.getGoalStatus().status != actionlib_msgs::GoalStatus::PREEMPTING)
    return;

  gh.setSucceeded(result);
}

void contoursAlgorithmCancelCb(actionlib::ServerGoalHandle<ram_path_planning::ContoursAction> gh)
{
  if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE
      || gh.getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTING)
    gh.setCanceled();
}

// Follow poses algorithm
void FollowPosesAlgorithmGoalCb(actionlib::ServerGoalHandle<ram_path_planning::FollowPosesAction> gh)
{
  // Change status to ACTIVE
  gh.setAccepted();
  // Action elements
  ram_path_planning::FollowPosesGoal goal;
  ram_path_planning::FollowPosesResult result;
  goal = *gh.getGoal();

  // Action feedback
  if (!follow_poses_generator->publishStatusPercentageDone("Checking goal parameters", 10, gh))
    return;

  if (goal.file.empty())
  {
    result.error_msg = "File name cannot be empty";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  // Verify file extension
  const std::string file_extension(ram_utils::fileExtension(goal.file));
  if (strcasecmp(file_extension.c_str(), "yaml") && strcasecmp(file_extension.c_str(), "yml")
      && strcasecmp(file_extension.c_str(), "svg"))
  {
    result.error_msg = "File is not a SVG, YAML, YML, " + goal.file;
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  // Action feedback
  if (!follow_poses_generator->publishStatusPercentageDone("Generating trajectory from SVG/YAML file", 40, gh))
    return;

  // Create Trajectory message
  ram_msgs::AdditiveManufacturingTrajectory msg;
  msg.file = goal.file;

  // Generate trajectory
  std::string error_message;
  error_message = follow_poses_generator->generateTrajectory(goal.file, msg, goal.arc_points, goal.rotate_poses);
  if (!error_message.empty())
  {
    result.error_msg = error_message;
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  // Add request parameter to the trajectory
  msg.generation_info += "Generation algorithm = Follow poses\n";
  std::stringstream value;
  value << std::fixed << std::setprecision(3) << goal.height_between_layers * 1000.0;
  std::string s(value.str());

  msg.generation_info += "Layer height = " + s + " mm\n";
  s.clear();
  value.str("");

  value << std::fixed << std::setprecision(3) << goal.number_of_layers * goal.height_between_layers * 1000.0;
  s = value.str();
  msg.generation_info += "Total height = " + s + " mm\n";

  if (!(strcasecmp(file_extension.c_str(), "svg")))
  {
    msg.generation_info += "Arc points = " + std::to_string(goal.arc_points) + "\n";
    msg.generation_info += "Rotate poses = " + std::to_string(goal.rotate_poses) + "\n";
  }
    msg.generation_info += "Number of layers = " + std::to_string(goal.number_of_layers) + "\n";
    std::string inverted_layer;
    (bool)goal.invert_one_of_two_layers ? inverted_layer = "true" : inverted_layer = "false";
    msg.generation_info += "Inverted one of two layers = " + inverted_layer + "\n";

    std::string duplicate_layer;
    (bool)goal.duplicate_layer ? duplicate_layer = "true" : duplicate_layer = "false";
    msg.generation_info += "Inverted one of two layers = " + duplicate_layer + "\n";

  if (msg.poses.empty())
  {
    result.error_msg = "Generated trajectory is empty";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }
  if (goal.duplicate_layer && msg.poses.back().layer_index != 0)
  {
    result.error_msg = "Trajectory contains multiple layers, cannot duplicate layers";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  unsigned polygon_count(0);
  for (auto pose : msg.poses)
  {
    if (pose.polygon_start)
      ++polygon_count;
  }

  if (goal.duplicate_layer && polygon_count == 0)
  {
    result.error_msg = "Trajectory contains zero polygon!";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.duplicate_layer && goal.number_of_layers > 1)
  {
    error_message = follow_poses_generator->duplicateLayers(msg, goal.number_of_layers, goal.height_between_layers,
                                                            goal.invert_one_of_two_layers);
    if (!error_message.empty())
    {
      result.error_msg = error_message + "\nError duplicating layers in the trajectory";
      ROS_ERROR_STREAM(result.error_msg);
      if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        gh.setAborted(result);
      return;
    }
  }

  // Trajectory is now complete
  // Create a message
  // Fill response and publish trajectory
  // Action feedback
  if (!follow_poses_generator->publishStatusPercentageDone("Creating message to publish trajectory from file", 70, gh))

    if (msg.poses.size() == 0)
    {
      result.error_msg = "Generated trajectory is empty";
      ROS_ERROR_STREAM(result.error_msg);
      if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        gh.setAborted(result);
      return;
    }
  result.number_of_poses = msg.poses.size();

  // The trajectory is incomplete at this stage
  // It will be published on trajectory_tmp, completed by fill_trajectory and then published on trajectory

  // Add UUIDs
  for (auto &pose : msg.poses)
    pose.unique_id = unique_id::toMsg(unique_id::fromRandom());

  // Action feedback
  if (!follow_poses_generator->publishStatusPercentageDone("Trajectory has been generated", 100, gh))
    return;

  if (traj_pub->getNumSubscribers() == 0)
    ROS_ERROR_STREAM("No subscriber on topic " << traj_pub->getTopic() << ": trajectory is lost");

  traj_pub->publish(msg);

  if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE
      && gh.getGoalStatus().status != actionlib_msgs::GoalStatus::PREEMPTING)
    return;

  gh.setSucceeded(result);
}

void FollowPosesAlgorithmCancelCb(actionlib::ServerGoalHandle<ram_path_planning::FollowPosesAction> gh)
{
  if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE
      || gh.getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTING)
    gh.setCanceled();
}

// Revolve algorithm
void RevolveAlgorithmGoalCb(actionlib::ServerGoalHandle<ram_path_planning::RevolveAction> gh)
{
  // Change status to ACTIVE
  gh.setAccepted();
  // Action elements
  ram_path_planning::RevolveGoal goal;
  ram_path_planning::RevolveResult result;
  goal = *gh.getGoal();

  // Action feedback
  if (!revolve_generator->publishStatusPercentageDone("Checking goal parameters", 10, gh))
    return;

  if (goal.file.empty())
  {
    result.error_msg = "File name cannot be empty";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }
  // Verify file extension
  const std::string file_extension(ram_utils::fileExtension(goal.file));
  if (strcasecmp(file_extension.c_str(), "svg"))
  {
    result.error_msg = "File is not a SVG, " + goal.file;
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.deposited_material_width <= 0.0)
  {
    result.error_msg = "Deposited material width must be > 0, " + std::to_string(goal.deposited_material_width);
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.height_between_layers <= 0)
  {
    result.error_msg = "Height between layers must be > 0, " + std::to_string(goal.height_between_layers);
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.number_of_passes == 0)
  {
    result.error_msg = "Number of passes cannot be zero, " + std::to_string(goal.number_of_passes);
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.revolution_number_of_points < 3)
  {
    result.error_msg = "Revolution number of points must be >= 3, " + std::to_string(goal.revolution_number_of_points);
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  // Action feedback
  if (!revolve_generator->publishStatusPercentageDone("Generating trajectory from SVG file", 40, gh))
    return;

  // Create Trajectory message
  ram_msgs::AdditiveManufacturingTrajectory msg;
  std::string error("");
  error = revolve_generator->generateTrajectory(gh,
                                                msg,
                                                goal.file,
                                                goal.deposited_material_width,
                                                goal.height_between_layers,
                                                goal.number_of_passes,
                                                goal.connection_angle,
                                                goal.revolution_number_of_points,
                                                goal.towards_interior,
                                                goal.slice_along_path,
                                                goal.arc_points);

  // Add information to the trajectory

  msg.generation_info = "Generation algorithm = Revolve\n";

  std::stringstream value;
  value << std::fixed << std::setprecision(3) << goal.height_between_layers * 1000.0;
  std::string s(value.str());
  msg.generation_info += "Layer height = " + s + " mm\n";
  s.clear();
  value.str("");

  auto last = std::prev(msg.poses.end());

  value << std::fixed << std::setprecision(3) << last->layer_level * goal.height_between_layers * 1000.0;
  s = value.str();
  msg.generation_info += "Total height = " + s + " mm\n";
  s.clear();
  value.str("");

  msg.generation_info += "Points per revolution = " + std::to_string(goal.revolution_number_of_points) + "\n";

  value << std::fixed << std::setprecision(3) << goal.number_of_passes * goal.deposited_material_width * 1000.0;
  s = value.str();
  msg.generation_info += "Total width = " + s + " mm\n";

  if (!error.empty())
  {
    result.error_msg = error;
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  // Add UUIDs
  for (auto &pose : msg.poses)
    pose.unique_id = unique_id::toMsg(unique_id::fromRandom());

  // Action feedback
  if (!revolve_generator->publishStatusPercentageDone("Trajectory has been generated", 100, gh))
    return;

  if (traj_pub->getNumSubscribers() == 0)
    ROS_ERROR_STREAM("No subscriber on topic " << traj_pub->getTopic() << ": trajectory is lost");

  traj_pub->publish(msg);

  if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE
      && gh.getGoalStatus().status != actionlib_msgs::GoalStatus::PREEMPTING)
    return;

  gh.setSucceeded(result);
}

void RevolveAlgorithmCancelCb(actionlib::ServerGoalHandle<ram_path_planning::RevolveAction> gh)
{
  if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE
      || gh.getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTING)
    gh.setCanceled();
}

// Polygon offsets algorithm
void PolygonOffsetsAlgorithmGoalCb(actionlib::ServerGoalHandle<ram_path_planning::PolygonOffsetsAction> gh)
{
  // Change status to ACTIVE
  gh.setAccepted();
  // Action elements
  ram_path_planning::PolygonOffsetsGoal goal;
  ram_path_planning::PolygonOffsetsResult result;
  goal = *gh.getGoal();

  // Action feedback
  if (!polygon_offsets_generator->publishStatusPercentageDone("Checking goal parameters", 10, gh))
    return;

  if (goal.file.empty())
  {
    result.error_msg = "File name cannot be empty";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  // Verify file extension
  const std::string file_extension(ram_utils::fileExtension(goal.file));
  if (strcasecmp(file_extension.c_str(), "yaml") && strcasecmp(file_extension.c_str(), "yml")
      && strcasecmp(file_extension.c_str(), "svg"))
  {
    result.error_msg = "File is not a SVG, YAML, YML, " + goal.file;
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.number_of_layers < 1)
  {
    result.error_msg = "Number of layers must be > 0, " + std::to_string(goal.number_of_layers);
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.deposited_material_width <= 0.0)
  {
    result.error_msg = "Deposited material width must be > 0, " + std::to_string(goal.deposited_material_width);
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.height_between_layers <= 0)
  {
    result.error_msg = "Height between layers must be > 0, " + std::to_string(goal.height_between_layers);
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.number_of_passes == 0)
  {
    result.error_msg = "Number of passes cannot be zero, " + std::to_string(goal.number_of_passes);
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.arc_tolerance <= 0)
  {
    result.error_msg = "Arc tolerance must be positive, " + std::to_string(goal.arc_tolerance);
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.miter_limit <= 0)
  {
    result.error_msg = "Miter limit must be positive, " + std::to_string(goal.miter_limit);
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.safe_distance < 0)
  {
    result.error_msg = "Can not compute if safe distance is null or negative, " + std::to_string(goal.safe_distance);
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.offset_factor < 0)
  {
    result.error_msg = "Can not compute if offset factor is null or negative, " + std::to_string(goal.offset_factor);
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  // Action feedback
  if (!(strcasecmp(file_extension.c_str(), "yaml") || strcasecmp(file_extension.c_str(), "yml")))
  {
    if (!polygon_offsets_generator->publishStatusPercentageDone("Generating trajectory from YAML file", 40, gh))
      return;
  }
  else if (!(strcasecmp(file_extension.c_str(), "svg")))
  {
    if (!polygon_offsets_generator->publishStatusPercentageDone("Generating trajectory from SVG file", 40, gh))
      return;
  }


  // Create Trajectory message
  ram_msgs::AdditiveManufacturingTrajectory msg;
  std::string error("");

  // Generate trajectory
  error = polygon_offsets_generator->generateTrajectory(
        gh,
        msg,
        goal.file,
        goal.number_of_layers,
        goal.height_between_layers,
        goal.deposited_material_width,
        goal.connection_value,
        goal.connection_type,
        goal.offset_factor,
        goal.safe_distance,
        goal.towards_interior,
        goal.number_of_passes,
        goal.join_type,
        goal.end_type,
        goal.arc_tolerance,
        goal.miter_limit,
        goal.arc_points,
        goal.discontinous_trajectory,
        goal.avoid_trajectories_crossing,
        goal.automatic_reverse_path,
        goal.reverse_origin_path);

  if (!error.empty())
  {
    result.error_msg = error;
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  // Add information to the trajectory
  msg.generation_info = "Generation algorithm = Polygon offsets\n";

  std::stringstream value;
  std::string s;

  value << std::fixed << std::setprecision(3) << goal.deposited_material_width * 1000.0;
  s = value.str();
  msg.generation_info += "Deposited material width = " + s + " mm\n";
  s.clear();
  value.str("");

  // Angle or distance connection display
  if (!goal.connection_type) // If the user enter only the connection angle
  {
    value << std::fixed << std::setprecision(1) << goal.connection_value * 180.0 / M_PI;
    s = value.str();
    msg.generation_info += "Connection mode = Angle\n";
    msg.generation_info += "Connection angle = " + s + " °\n";
    s.clear();
    value.str("");
  }
  else // If the user enter only the connection distance
  {
    value << std::fixed << std::setprecision(2) << goal.connection_value * 1000;
    s = value.str();
    msg.generation_info += "Connection mode = Distance\n";
    msg.generation_info += "Connection distance = " + s + " mm\n";
    s.clear();
    value.str("");
  }

  value << std::fixed << goal.number_of_passes;
  s = value.str();
  msg.generation_info += "Number of passes = " + s + "\n";
  s.clear();
  value.str("");

  std::string interior_str;
  goal.towards_interior ? interior_str = "true" : interior_str = "false";
  msg.generation_info += "Toward interior = " + interior_str + "\n";
  s.clear();
  value.str("");

  value << std::fixed << std::setprecision(2) << goal.arc_tolerance;
  s = value.str();
  msg.generation_info += "Arc tolerance = " + s + "\n";
  s.clear();
  value.str("");

  value << std::fixed << std::setprecision(2) << goal.miter_limit;
  s = value.str();
  msg.generation_info += "Miter limit = " + s + "\n";
  s.clear();
  value.str("");

  value << std::fixed << goal.arc_points;
  s = value.str();
  msg.generation_info += "Arc points = " + s + "\n";
  s.clear();
  value.str("");

  value << std::fixed << goal.number_of_layers;
  s = value.str();
  msg.generation_info += "Number of layers = " + s + "\n";
  s.clear();
  value.str("");

  std::string joint_type_str;
  switch(goal.join_type)
  {
    case 0:
      joint_type_str = "Square";
      break;
    case 1:
      joint_type_str = "Round";
      break;
    case 2:
      joint_type_str = "Miter";
      break;
    default:
      joint_type_str = "Error : join type cannot be read";
      break;
  }
  msg.generation_info += "Join type = " + joint_type_str + "\n";
  s.clear();
  value.str("");

  std::string end_type_str;
  switch(goal.end_type)
  {
    case 0:
      end_type_str = "Closed polygon";
      break;
    case 1:
      end_type_str = "Closed line";
      break;
    case 2:
      end_type_str = "Open butt";
      break;
    case 3:
      end_type_str = "Open square";
      break;
    case 4:
      end_type_str = "Open round";
      break;
    default:
      joint_type_str = "Error : end type cannot be read";
      break;
  }
  msg.generation_info += "End type = " + end_type_str + "\n";
  s.clear();
  value.str("");

  value << std::fixed << std::setprecision(3) << goal.height_between_layers * 1000.0;
  s = value.str();
  msg.generation_info += "Layer height = " + s + " mm\n";
  s.clear();
  value.str("");

  auto last = std::prev(msg.poses.end());
  long last_level_tmp(last->layer_level);
  last_level_tmp == 0 ? last_level_tmp = 1 : last_level_tmp = last->layer_level;

  value << std::fixed << std::setprecision(3) << last_level_tmp * goal.height_between_layers * 1000.0;
  s = value.str();
  msg.generation_info += "Total height = " + s + " mm\n";
  s.clear();
  value.str("");

  // Add UUIDs
  for (auto &pose : msg.poses)
    pose.unique_id = unique_id::toMsg(unique_id::fromRandom());

  // Action feedback
  if (!polygon_offsets_generator->publishStatusPercentageDone("Trajectory has been generated", 100, gh))
    return;

  if (traj_pub->getNumSubscribers() == 0)
    ROS_ERROR_STREAM("No subscriber on topic " << traj_pub->getTopic() << ": trajectory is lost");

  traj_pub->publish(msg);

  if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE
      && gh.getGoalStatus().status != actionlib_msgs::GoalStatus::PREEMPTING)
    return;

  gh.setSucceeded(result);
}

void PolygonOffsetsAlgorithmCancelCb(actionlib::ServerGoalHandle<ram_path_planning::PolygonOffsetsAction> gh)
{
  if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE
      || gh.getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTING)
    gh.setCanceled();
}

// Profile algorithm
void ProfileAlgorithmGoalCb(actionlib::ServerGoalHandle<ram_path_planning::ProfileAction> gh)
{
  // Change status to ACTIVE
  gh.setAccepted();
  // Action elements
  ram_path_planning::ProfileGoal goal;
  ram_path_planning::ProfileResult result;
  goal = *gh.getGoal();

  // Action feedback
  if (!profile_generator->publishStatusPercentageDone("Checking goal parameters", 10, gh))
    return;

  if (goal.file.empty())
  {
    result.error_msg = "File name cannot be empty";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }
  // Verify file extension
  const std::string file_extension(ram_utils::fileExtension(goal.file));
  if (strcasecmp(file_extension.c_str(), "svg"))
  {
    result.error_msg = "File is not a SVG, " + goal.file;
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.deposited_material_width <= 0.0)
  {
    result.error_msg = "Deposited material width must be > 0, " + std::to_string(goal.deposited_material_width);
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.height_between_layers <= 0)
  {
    result.error_msg = "Height between layers must be > 0, " + std::to_string(goal.height_between_layers);
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.number_of_passes == 0)
  {
    result.error_msg = "Number of passes cannot be zero, " + std::to_string(goal.number_of_passes);
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.angle_percentage < 0.0 || goal.angle_percentage > 1.0)
  {
    result.error_msg = "Angle percentage must be between 0.0 and 1.0. " + std::to_string(goal.angle_percentage);
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  // Action feedback
  if (!profile_generator->publishStatusPercentageDone("Generating trajectory from SVG file", 40, gh))
    return;

  // Create Trajectory message
  ram_msgs::AdditiveManufacturingTrajectory msg;
  std::string error("");
  error = profile_generator->generateTrajectory(gh,
                                                msg,
                                                goal.file,
                                                goal.deposited_material_width,
                                                goal.height_between_layers,
                                                goal.number_of_passes,
                                                goal.towards_interior,
                                                goal.slice_along_path,
                                                goal.angle_percentage,
                                                goal.angle_type,
                                                goal.arc_points);


  // Add information to the trajectory
  msg.generation_info = "Generation algorithm = Profile\n";

  std::stringstream value;
  value << std::fixed << std::setprecision(3) << goal.height_between_layers * 1000.0;
  std::string s(value.str());
  msg.generation_info += "Layer height = " + s + " mm\n";
  s.clear();
  value.str("");

  auto last = std::prev(msg.poses.end());

  value << std::fixed << std::setprecision(3) << last->layer_level * goal.height_between_layers * 1000.0;
  s = value.str();
  msg.generation_info += "Total height = " + s + " mm\n";
  s.clear();
  value.str("");

  if (!error.empty())
  {
    result.error_msg = error;
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  // Add UUIDs
  for (auto &pose : msg.poses)
    pose.unique_id = unique_id::toMsg(unique_id::fromRandom());

  // Action feedback
  if (!profile_generator->publishStatusPercentageDone("Trajectory has been generated", 100, gh))
    return;

  if (traj_pub->getNumSubscribers() == 0)
    ROS_ERROR_STREAM("No subscriber on topic " << traj_pub->getTopic() << ": trajectory is lost");

  traj_pub->publish(msg);

  if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE
      && gh.getGoalStatus().status != actionlib_msgs::GoalStatus::PREEMPTING)
    return;

  gh.setSucceeded(result);
}

void ProfileAlgorithmCancelCb(actionlib::ServerGoalHandle<ram_path_planning::ProfileAction> gh)
{
  if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE
      || gh.getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTING)
    gh.setCanceled();
}

int main(int argc,
         char **argv)
{
  ros::init(argc, argv, "path_planning");
  donghongding_generator.reset(new ram_path_planning::DonghongDing<ram_path_planning::DonghongDingAction>);
  contour_generator.reset(new ram_path_planning::Contours<ram_path_planning::ContoursAction>);
  follow_poses_generator.reset(new ram_path_planning::FollowPoses<ram_path_planning::FollowPosesAction>);
  revolve_generator.reset(new ram_path_planning::Revolve<ram_path_planning::RevolveAction>);
  polygon_offsets_generator.reset(new ram_path_planning::PolygonOffsets<ram_path_planning::PolygonOffsetsAction>);
  profile_generator.reset(new ram_path_planning::Profile<ram_path_planning::ProfileAction>);

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  traj_pub.reset(new ros::Publisher);
  *traj_pub = nh.advertise<ram_msgs::AdditiveManufacturingTrajectory>("ram/trajectory_tmp", 1, true);

  // Action servers
  actionlib::ActionServer<ram_path_planning::DonghongDingAction> action_server_1(nh,
                                                                                 donghongding_generator->service_name_,
                                                                                 &donghongDingAlgorithmGoalCb,
                                                                                 &donghongDingAlgorithmCancelCb,
                                                                                 false);
  action_server_1.start();

  actionlib::ActionServer<ram_path_planning::ContoursAction> action_server_2(nh, contour_generator->service_name_,
                                                                             &contoursAlgorithmGoalCb,
                                                                             &contoursAlgorithmCancelCb, false);
  action_server_2.start();

  actionlib::ActionServer<ram_path_planning::FollowPosesAction> action_server_3(nh,
                                                                                follow_poses_generator->service_name_,
                                                                                &FollowPosesAlgorithmGoalCb,
                                                                                &FollowPosesAlgorithmCancelCb,
                                                                                false);
  action_server_3.start();

  actionlib::ActionServer<ram_path_planning::RevolveAction> action_server_4(nh,
                                                                            revolve_generator->service_name_,
                                                                            &RevolveAlgorithmGoalCb,
                                                                            &RevolveAlgorithmCancelCb,
                                                                            false);
  action_server_4.start();

  actionlib::ActionServer<ram_path_planning::PolygonOffsetsAction> action_server_5(nh,
                                                                                   polygon_offsets_generator->service_name_,
                                                                                   &PolygonOffsetsAlgorithmGoalCb,
                                                                                   &PolygonOffsetsAlgorithmCancelCb,
                                                                                   false);

  action_server_5.start();

  actionlib::ActionServer<ram_path_planning::ProfileAction> action_server_6(nh,
                                                                            profile_generator->service_name_,
                                                                            &ProfileAlgorithmGoalCb,
                                                                            &ProfileAlgorithmCancelCb,
                                                                            false);
  action_server_6.start();

  nh.param<bool>("ram/path_planning/use_gui", use_gui, false);
  if (use_gui)
  {
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> render_window = vtkSmartPointer<vtkRenderWindow>::New();
    render_window->AddRenderer(renderer);
    render_window->SetSize(800, 600);

    vtkSmartPointer<vtkRenderWindowInteractor> render_window_interactor =
        vtkSmartPointer<vtkRenderWindowInteractor>::New();
    render_window_interactor->SetRenderWindow(render_window);
    render_window_interactor->Initialize();

    vtkSmartPointer<vtkInteractorStyleTrackballCamera> image_style =
        vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    render_window_interactor->SetInteractorStyle(image_style);

    render_window_interactor->AddObserver(vtkCommand::TimerEvent, cb);
    render_window_interactor->CreateRepeatingTimer(500);

    render_window_interactor->Start();
  }

  ros::waitForShutdown();
  spinner.stop();
  traj_pub.reset();
  return 0;
}
