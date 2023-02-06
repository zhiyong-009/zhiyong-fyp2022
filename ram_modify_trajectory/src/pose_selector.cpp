#include <mutex>
#include <string>
#include <strings.h>
#include <algorithm>

#include <ros/ros.h>

#include <unique_id/unique_id.h>
#include <uuid_msgs/UniqueID.h>

#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_msgs/AdditiveManufacturingPose.h>

#include <ram_modify_trajectory/AddToSelection.h>
#include <ram_modify_trajectory/EraseSelection.h>
#include <ram_modify_trajectory/GetPosesFromInformation.h>
#include <ram_modify_trajectory/GetPosesFromLayer.h>
#include <ram_modify_trajectory/GetPosesFromLayersList.h>
#include <ram_modify_trajectory/GetPosesFromTrajectory.h>
#include <ram_modify_trajectory/GetSelection.h>
#include <ram_modify_trajectory/InvertSelection.h>
#include <ram_modify_trajectory/RemoveFromSelection.h>

std::mutex trajectory_mutex;
ram_msgs::AdditiveManufacturingTrajectory layers;

std::mutex selection_params_mutex;
std::vector<ram_msgs::AdditiveManufacturingPose> selection;

void trajectoryInfoCallback(const ram_msgs::AdditiveManufacturingTrajectoryConstPtr &msg)
{
  std::lock_guard<std::mutex> lock_1(trajectory_mutex);
  std::lock_guard<std::mutex> lock_2(selection_params_mutex);

  layers = *msg;
  selection.clear();
}

bool getPosesFromTrajectoryCallback(ram_modify_trajectory::GetPosesFromTrajectory::Request &req,
                                    ram_modify_trajectory::GetPosesFromTrajectory::Response &res)
{
  std::lock_guard<std::mutex> lock(trajectory_mutex);
  res.poses.clear();

  if (layers.poses.empty())
  {
    res.error = "Trajectory is empty";
    return true;
  }

  if (req.pose_index_list.empty())
  {
    res.error = "Pose list is empty";
    return true;
  }

  std::vector<unsigned> pose_index_list = req.pose_index_list;
  std::sort(pose_index_list.begin(), pose_index_list.end()); // Sort the list of index

  if (pose_index_list.back() >= layers.poses.size())
    return true;

  for (auto pose_index : pose_index_list)
    res.poses.push_back(layers.poses[pose_index]);

  return true;
}

bool getPosesFromLayersListCallback(ram_modify_trajectory::GetPosesFromLayersList::Request &req,
                                    ram_modify_trajectory::GetPosesFromLayersList::Response &res)
{
  std::lock_guard<std::mutex> lock(trajectory_mutex);

  if (layers.poses.empty())
  {
    res.error = "Trajectory is empty";
    return true;
  }

  if (req.layer_level_list.empty())
  {
    res.error = "Layer level list is empty";
    return true;
  }

  if (req.layer_level_list.back() > layers.poses.back().layer_level)
  {
    res.error = "Layer level requested is higher than the pose layer level";
    return true;
  }

  // Find poses
  for (auto pose : layers.poses)
  {
    for (auto current_layer_level : req.layer_level_list)
    {
      if (pose.layer_level != current_layer_level)
        continue;

      res.poses.push_back(pose);
      break;
    }
  }
  return true;
}

bool getPosesFromLayerCallback(ram_modify_trajectory::GetPosesFromLayer::Request &req,
                               ram_modify_trajectory::GetPosesFromLayer::Response &res)
{
  std::lock_guard<std::mutex> lock(trajectory_mutex);

  if (layers.poses.empty())
  {
    res.error = "Trajectory is empty";
    return true;
  }

  if (req.index_list_relative.empty())
  {
    res.error = "Relative index list list is empty";
    return true;
  }

  if (req.layer_level > layers.poses.back().layer_level)
  {
    res.error = "Layer level requested is higher than the pose layer level";
    return true;
  }

  std::vector<unsigned> index_list_relative = req.index_list_relative;
  std::sort(index_list_relative.begin(), index_list_relative.end()); // Sort the list of index

  // Copy the poses of the layer in a new vector
  std::vector<ram_msgs::AdditiveManufacturingPose> layer;
  for (auto &pose : layers.poses)
    if (pose.layer_level == req.layer_level)
      layer.push_back(pose);

  if (layer.empty())
  {
    res.error = "Layer is empty";
    return true;
  }

  if (index_list_relative.back() >= layer.size())
  {
    res.error = "There are invalid indices in the request";
    return true;
  }

  // Save the poses
  for (unsigned local_index(0); local_index < layer.size(); ++local_index)
  {
    if (index_list_relative.empty())
      break;

    if (local_index == index_list_relative.front())
    {
      res.poses.push_back(layer[local_index]); // Previous checks makes sure this pose exists
      index_list_relative.erase(index_list_relative.begin());
    }

  }
  return true;
}

bool getPosesFromInformationCallback(ram_modify_trajectory::GetPosesFromInformation::Request &req,
                                     ram_modify_trajectory::GetPosesFromInformation::Response &res)
{
  std::lock_guard<std::mutex> lock(trajectory_mutex);
  selection.clear();

  if (layers.poses.empty())
  {
    res.error = "Input trajectory is empty";
    return true;
  }

  if (!req.information_parameters.polygon_start && !req.information_parameters.polygon_end
      && !req.information_parameters.both_start_end && !req.information_parameters.last_pose_in_layer)
  {
    res.error = "Please select one of these criteria";
    return true;
  }

  if ((req.information_parameters.polygon_start || req.information_parameters.polygon_end)
      && req.information_parameters.both_start_end)
  {
    res.error = "It is not possible to filter only polygon start or end and both criteria";
    return true;
  }

  if ((req.information_parameters.polygon_start && req.information_parameters.polygon_end))
  {
    res.error = "Please filter with the third option (both)";
    return true;
  }

  if (req.information_parameters.polygon_start)
  {
    for (auto &pose : layers.poses)
    {
      if ((pose.polygon_start && !pose.polygon_end))
        selection.emplace_back(pose);
    }
  }
  else if (req.information_parameters.polygon_end)
  {
    for (auto &pose : layers.poses)
    {
      if ((pose.polygon_end && !pose.polygon_start))
        selection.emplace_back(pose);
    }
  }
  else if (req.information_parameters.both_start_end)
  {
    for (auto &pose : layers.poses)
    {
      if ((pose.polygon_start && pose.polygon_end))
        selection.emplace_back(pose);
    }
  }
  else if (req.information_parameters.last_pose_in_layer)
  {
    using PoseIt = std::vector<ram_msgs::AdditiveManufacturingPose>::iterator;
    PoseIt last(layers.poses.begin());
    std::size_t layer_index(layers.poses.at(0).layer_index);

    for (PoseIt it(layers.poses.begin()); it != layers.poses.end(); ++it)
    {
      if (it->layer_index == layer_index)
      {
        last = it;
        continue;
      }

      selection.emplace_back(*it);
      last = it;
      layer_index = it->layer_index;
    }
  }

  res.selection = selection;
  return true;
}

bool addToSelectionCallback(ram_modify_trajectory::AddToSelection::Request &req,
                            ram_modify_trajectory::AddToSelection::Response &res)
{
  std::lock_guard<std::mutex> lock(selection_params_mutex);

  unsigned selection_size = selection.size(); // size before insert new poses

  for (auto request_pose : req.poses)
  {
    bool is_duplicate = false;

    // verify duplicate
    for (unsigned i(0); i < selection_size; ++i)
    {
      if (request_pose.unique_id.uuid != selection[i].unique_id.uuid) // uuid are not equals
        continue;

      is_duplicate = true;
      break;
    }

    if (!is_duplicate)
      selection.push_back(request_pose);
  }
  res.selection = selection;
  return true;
}

bool removeFromSelectionCallback(ram_modify_trajectory::RemoveFromSelection::Request &req,
                                 ram_modify_trajectory::RemoveFromSelection::Response &res)
{
  std::lock_guard<std::mutex> lock(selection_params_mutex);

  for (auto request_pose : req.poses)
  {
    // verify duplicate
    for (unsigned i(0); i < selection.size(); ++i)
    {
      if (request_pose.unique_id.uuid != selection[i].unique_id.uuid) // uuid are not equals
        continue;

      selection.erase(selection.begin() + i);
      break;
    }
  }
  res.selection = selection;
  return true;
}

bool getSelectionCallback(ram_modify_trajectory::GetSelection::Request &,
                          ram_modify_trajectory::GetSelection::Response &res)
{
  std::lock_guard<std::mutex> lock(selection_params_mutex);

  if (!res.selection.empty())
  {
    res.selection = selection;
    return true;
  }

  res.error = "Output selection is empty";
  return true;
}

bool eraseSelectionCallback(ram_modify_trajectory::EraseSelection::Request &,
                            ram_modify_trajectory::EraseSelection::Response &res)
{
  std::lock_guard<std::mutex> lock(selection_params_mutex);
  selection.clear();

  if (!res.selection.empty())
  {
    res.error = "Cannot clean the selection";
    return true;
  }
    res.selection = selection;
    return true;
}

bool invertSelectionCallback(ram_modify_trajectory::InvertSelection::Request &req,
                             ram_modify_trajectory::InvertSelection::Response &res)
{
  std::lock_guard<std::mutex> lock(selection_params_mutex);

  for (auto request_pose : req.poses)
  {
    bool is_duplicate = false;
    // verify duplicate
    for (unsigned i(0); i < selection.size(); ++i)
    {
      if (selection[i].unique_id.uuid != request_pose.unique_id.uuid) // uuid are not equals
        continue;

      is_duplicate = true;
      selection.erase(selection.begin() + i); // Remove
      break;
    }

    if (!is_duplicate)
      selection.push_back(request_pose); // Add
  }
  res.selection = selection;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_selector");
  ros::NodeHandle nh;

  // Subscribe on "ram/trajectory"
  ros::Subscriber sub = nh.subscribe("ram/trajectory", 10, trajectoryInfoCallback);

  // Select poses
  ros::ServiceServer service_1 = nh.advertiseService("ram/pose_selector/get_poses_from_trajectory",
                                                     getPosesFromTrajectoryCallback);
  ros::ServiceServer service_2 = nh.advertiseService("ram/pose_selector/get_poses_from_layers_list",
                                                     getPosesFromLayersListCallback);
  ros::ServiceServer service_3 = nh.advertiseService("ram/pose_selector/get_poses_from_layer",
                                                     getPosesFromLayerCallback);
  ros::ServiceServer service_4 = nh.advertiseService("ram/pose_selector/get_poses_from_information",
                                                     getPosesFromInformationCallback);
  // Modify Selection
  ros::ServiceServer service_5 = nh.advertiseService("ram/pose_selector/add_to_selection", addToSelectionCallback);
  ros::ServiceServer service_6 = nh.advertiseService("ram/pose_selector/remove_from_selection",
                                                     removeFromSelectionCallback);
  ros::ServiceServer service_7 = nh.advertiseService("ram/pose_selector/get_selection", getSelectionCallback);
  ros::ServiceServer service_8 = nh.advertiseService("ram/pose_selector/erase_selection", eraseSelectionCallback);
  ros::ServiceServer service_9 = nh.advertiseService("ram/pose_selector/invert_selection", invertSelectionCallback);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
