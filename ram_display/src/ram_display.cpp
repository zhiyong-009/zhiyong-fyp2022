#include <eigen_conversions/eigen_msg.h>
#include <interactive_markers/interactive_marker_server.h>
#include <mutex>
#include <ram_display/DeleteTrajectory.h>
#include <ram_display/DisplayRangeOfLayers.h>
#include <ram_display/DisplayTrajectory.h>
#include <ram_display/UpdateMeshColor.h>
#include <ram_display/UpdateSelection.h>
#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_utils/file_extension.hpp>
#include <ram_utils/GetTool.h>
#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <sstream>
#include <std_msgs/ColorRGBA.h>
#include <string>

ros::Publisher mesh_pub;

rviz_visual_tools::RvizVisualToolsPtr rvt_trajectory;
rviz_visual_tools::RvizVisualToolsPtr rvt_selection;

std::string trajectory_frame;

// Get tool service client
ros::ServiceClient get_tool_client;

// Trajectory
std::recursive_mutex trajectory_mutex;
ram_msgs::AdditiveManufacturingTrajectory trajectory;

// Display parameters
std::recursive_mutex display_params_mutex;
ram_display::DisplayTrajectory::Request display_params;

// Display parameters
std::recursive_mutex range_of_layers_params_mutex;
ram_display::DisplayRangeOfLayers::Request range_of_layers_params;

// Selection
std::recursive_mutex selection_params_mutex;
ram_display::UpdateSelection::Request selection;

bool updateSelection(bool display = true);

template<typename T>
std::string toStringWithPrecision(const T value, const unsigned p = 2)
{
  std::ostringstream o;
  o.precision(p);
  o << std::fixed << value;
  return o.str();
}

std_msgs::ColorRGBA intToColor(const unsigned int color)
{
  std_msgs::ColorRGBA rvalue;
  rvalue.a = 1;

  switch (color % 28)
  {
    case 0:
      rvalue.r = 225;
      rvalue.g = 11;
      rvalue.b = 29;
      break;
    case 1:
      rvalue.r = 0;
      rvalue.g = 136;
      rvalue.b = 31;
      break;
    case 2:
      rvalue.r = 143;
      rvalue.g = 63;
      rvalue.b = 250;
      break;
    case 3:
      rvalue.r = 0;
      rvalue.g = 172;
      rvalue.b = 197;
      break;
    case 4:
      rvalue.r = 255;
      rvalue.g = 128;
      rvalue.b = 207;
      break;
    case 5:
      rvalue.r = 139;
      rvalue.g = 254;
      rvalue.b = 60;
      break;
    case 6:
      rvalue.r = 113;
      rvalue.g = 8;
      rvalue.b = 78;
      break;
    case 7:
      rvalue.r = 255;
      rvalue.g = 165;
      rvalue.b = 66;
      break;
    case 8:
      rvalue.r = 0;
      rvalue.g = 13;
      rvalue.b = 154;
      break;
    case 9:
      rvalue.r = 136;
      rvalue.g = 112;
      rvalue.b = 105;
      break;
    case 10:
      rvalue.r = 0;
      rvalue.g = 73;
      rvalue.b = 66;
      break;
    case 11:
      rvalue.r = 0;
      rvalue.g = 253;
      rvalue.b = 208;
      break;
    case 12:
      rvalue.r = 82;
      rvalue.g = 42;
      rvalue.b = 14;
      break;
    case 13:
      rvalue.r = 188;
      rvalue.g = 183;
      rvalue.b = 252;
      break;
    case 14:
      rvalue.r = 146;
      rvalue.g = 180;
      rvalue.b = 125;
      break;
    case 15:
      rvalue.r = 200;
      rvalue.g = 18;
      rvalue.b = 182;
      break;
    case 16:
      rvalue.r = 0;
      rvalue.g = 103;
      rvalue.b = 160;
      break;
    case 17:
      rvalue.r = 41;
      rvalue.g = 6;
      rvalue.b = 64;
      break;
    case 18:
      rvalue.r = 224;
      rvalue.g = 179;
      rvalue.b = 176;
      break;
    case 19:
      rvalue.r = 255;
      rvalue.g = 245;
      rvalue.b = 152;
      break;
    case 20:
      rvalue.r = 81;
      rvalue.g = 69;
      rvalue.b = 90;
      break;
    case 21:
      rvalue.r = 168;
      rvalue.g = 124;
      rvalue.b = 35;
      break;
    case 22:
      rvalue.r = 255;
      rvalue.g = 114;
      rvalue.b = 106;
      break;
    case 23:
      rvalue.r = 51;
      rvalue.g = 129;
      rvalue.b = 111;
      break;
    case 24:
      rvalue.r = 136;
      rvalue.g = 7;
      rvalue.b = 21;
      break;
    case 25:
      rvalue.r = 166;
      rvalue.g = 124;
      rvalue.b = 177;
      break;
    case 26:
      rvalue.r = 49;
      rvalue.g = 78;
      rvalue.b = 19;
      break;
    default: // case 28
      rvalue.r = 144;
      rvalue.g = 228;
      rvalue.b = 253;
      break;
  }

  rvalue.r /= 255.0;
  rvalue.g /= 255.0;
  rvalue.b /= 255.0;
  return rvalue;
}

void displayMesh(const std::string file_name,
                 const std_msgs::ColorRGBA color = std_msgs::ColorRGBA(),
                 const std::string frame_id = "base")
{
  std::string file_extension(ram_utils::fileExtension(file_name));
  visualization_msgs::Marker mesh_marker;
  mesh_marker.pose.orientation.w = 1;

  if (!file_name.empty() && (!strcasecmp(file_extension.c_str(), "obj") ||
                             !strcasecmp(file_extension.c_str(), "ply") ||
                             !strcasecmp(file_extension.c_str(), "stl")))
  {
    mesh_marker.header.frame_id = frame_id;
    mesh_marker.header.stamp = ros::Time();
    mesh_marker.ns = "mesh";
    mesh_marker.id = 0;
    mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh_marker.action = visualization_msgs::Marker::ADD;
    mesh_marker.pose.position.x = 0;
    mesh_marker.pose.position.y = 0;
    mesh_marker.pose.position.z = 0;
    mesh_marker.scale.x = 1;
    mesh_marker.scale.y = 1;
    mesh_marker.scale.z = 1;
    mesh_marker.color = color;
    mesh_marker.mesh_resource = "file://" + file_name;
    mesh_marker.frame_locked = true;
  }
  else
  {
    // Delete mesh
    mesh_marker.header.frame_id = frame_id;
    mesh_marker.header.stamp = ros::Time();
    mesh_marker.ns = "mesh";
    mesh_marker.id = 0;
    mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh_marker.action = visualization_msgs::Marker::DELETE;
  }

  if (mesh_pub.getNumSubscribers() == 0)
    ROS_ERROR_STREAM("No subscriber on topic " << mesh_pub.getTopic() << ": mesh will not be displayed");
  mesh_pub.publish(mesh_marker);
}

void deleteTrajectory()
{
  displayMesh("");
  rvt_trajectory->deleteAllMarkers();
  rvt_trajectory->trigger();
  updateSelection(false); // Don't clear the selection, but don't display it!
}

std::string displayTrajectory()
{
  using AdditiveManufacturingPoseIt = std::vector<ram_msgs::AdditiveManufacturingPose>::iterator;
  std::lock_guard<std::recursive_mutex> lock_1(trajectory_mutex);
  std::lock_guard<std::recursive_mutex> lock_2(display_params_mutex);

  // Validate parameters
  if (trajectory.poses.size() < 2)
    return "Trajectory is too small. Two poses are required";

  if (display_params.display_type > 1)
    return "Value is out of range (0 - 1). display_type = " + std::to_string(display_params.display_type);

  if (display_params.line_size <= 0)
    return "Value is out of range (positive non null). line_size = " + std::to_string(display_params.line_size);

  if (display_params.color_mode > 8)
    return "Value is out of range (0 - 8). color_mode = " + std::to_string(display_params.color_mode);

  if (display_params.display_axis && display_params.axis_size <= 0)
    return "Value is out of range (positive non null). axis_size = " + std::to_string(display_params.axis_size);

  if (display_params.display_labels && display_params.label_size <= 0)
    return "Value is out of range (positive non null). label_size = " + std::to_string(display_params.label_size);

  if (display_params.label_type > 3)
    return "Value is out of range (0 - 3). label_type = " + std::to_string(display_params.label_type);

  std::lock_guard<std::recursive_mutex> lock_range(range_of_layers_params_mutex);
  if (range_of_layers_params.display_range_of_layers)
  {
    if (range_of_layers_params.last_layer < range_of_layers_params.first_layer)
      return "last_layer must be greater than first_layer";

    unsigned highest_level(trajectory.poses.front().layer_level);
    for (auto ram_pose : trajectory.poses)
    {
      if (ram_pose.layer_level > highest_level)
        highest_level = ram_pose.layer_level;
    }

    if (range_of_layers_params.last_layer > highest_level)
      return "Cannot display this range of layers, last layer do not exist in the trajectory.\n"
             "Try with a lower \"Last layer\" value.";
  }

  deleteTrajectory();

  // Display mesh if any
  displayMesh(trajectory.file,
              display_params.mesh_color,
              trajectory_frame);


    auto first_pose_displayed(trajectory.poses.begin()), last_pose_displayed(trajectory.poses.end());

  if (range_of_layers_params.display_range_of_layers)
  {
    first_pose_displayed = std::find_if(trajectory.poses.begin(), trajectory.poses.end(), [ = ](const ram_msgs::AdditiveManufacturingPose & p)
    {
      return (p.layer_index == range_of_layers_params.first_layer);
    });

    last_pose_displayed = std::find_if(trajectory.poses.begin(), trajectory.poses.end(), [ = ](const ram_msgs::AdditiveManufacturingPose & p)
    {
      return (p.layer_index == range_of_layers_params.last_layer + 1);
    });
  }

  EigenSTL::vector_Vector3d path;
  for (auto it(first_pose_displayed); it != last_pose_displayed; ++it)
  {
    Eigen::Isometry3d eigen_pose;
    tf::poseMsgToEigen(it->pose, eigen_pose);
    path.emplace_back(eigen_pose.translation());
  }


  // Publish X/Y/Z axis
  if (display_params.display_axis)
  {
    // Pos needs to be transformed to take account of tool orientation
    // Check that service exists
    if (!get_tool_client.waitForExistence(ros::Duration(0.5)))
      return "Cannot get tool, service does not exist";

    // Call service to get start pose
    ram_utils::GetTool srv;
    if (!get_tool_client.call(srv))
      return "Cannot get tool, call to service failed";

    Eigen::Isometry3d tool;
    tf::poseMsgToEigen(srv.response.pose, tool);

    EigenSTL::vector_Isometry3d path;
    for (auto it(first_pose_displayed); it != last_pose_displayed; ++it)
    {
      Eigen::Isometry3d pose;
      tf::poseMsgToEigen(it->pose, pose);
      pose = pose * tool;
      path.emplace_back(pose);
    }
    rvt_trajectory->publishAxisPath(path, display_params.axis_size, 0.1 * display_params.axis_size);
  }



  std::vector<std_msgs::ColorRGBA> colors;
  std::map<double, std_msgs::ColorRGBA> color_map;
  unsigned color(0);
  for (auto it(first_pose_displayed); it != last_pose_displayed; ++it)
  {
    switch (display_params.color_mode)
    {
      case 0: // Layer level
        {
          colors.emplace_back(intToColor(it->layer_level));
          break;
        }
      case 1: // Layer index
        {
          colors.emplace_back(intToColor(it->layer_index));
          break;
        }
      case 2: // Pose type
        {
          int color;
          if (it->entry_pose)
            color = 0;
          else if (it->exit_pose)
            color = 1;
          else if (it->polygon_start)
            color = 2;
          else if (it->polygon_end)
            color = 3;
          else
            color = 4;

          colors.emplace_back(intToColor(color));
          break;
        }
      case 3: // Movement type
        {
          int color;
          if (it->params.movement_type == 0)
            color = 0;
          else
            color = 1;

          colors.emplace_back(intToColor(color));
          break;
        }
      case 4: // Approach type
        {
          int color;
          if (it->params.approach_type == 0)
            color = 0;
          else
            color = 1;

          colors.emplace_back(intToColor(color));
          break;
        }
      case 5: // Blend radius
        {
          if (color_map.find(it->params.blend_radius) == color_map.end()) // Cannot find color
            color_map[it->params.blend_radius] = intToColor(color++);
          colors.emplace_back(color_map[it->params.blend_radius]);
          break;
        }
      case 6: // Speed
        {
          if (color_map.find(it->params.speed) == color_map.end()) // Cannot find color
            color_map[it->params.speed] = intToColor(color++);
          colors.emplace_back(color_map[it->params.speed]);
          break;
        }
      case 7: // Laser power
        {
          if (color_map.find(it->params.laser_power) == color_map.end()) // Cannot find color
            color_map[it->params.laser_power] = intToColor(color++);
          colors.emplace_back(color_map[it->params.laser_power]);
          break;
        }
      default: // Feed rate
        {
          if (color_map.find(it->params.feed_rate) == color_map.end()) // Cannot find color
            color_map[it->params.feed_rate] = intToColor(color++);
          colors.emplace_back(color_map[it->params.feed_rate]);
          break;
        }
    }
  }

  // New alpha trajectory application
  for (auto it(first_pose_displayed); it != last_pose_displayed; ++it)
  {
    auto i(std::distance(first_pose_displayed, it));

    if (it->entry_pose)
      colors.at(i).a = display_params.entry_pose_alpha;
    else if (it->exit_pose || it->polygon_end)
      colors.at(i).a = display_params.exit_pose_alpha;
    else
      colors.at(i).a = display_params.additive_pose_alpha;
  }

  // For speed color code only : remove the first color and duplicate the last color in order to shift the color vector
  // and to be in line with the speed modificator
  if (display_params.display_type && display_params.color_mode == 6)
  {
    colors.erase(colors.begin());
    colors.emplace_back(colors.back());
  }
  // Else: display trajectory for other color modes

  if (!display_params.display_type) // For cylinders
  {
    if (display_params.color_mode != 6) // Display trajectory with cylinders for all color modes, except for speed mode
    {
      if (path.size() < 2)
        return "Invalid path because not enough points are passed in";

      if (path.size() != colors.size())
        return "Invalid path because " + std::to_string(path.size()) + " different from " +
               std::to_string(colors.size());

      for (auto pose_it(first_pose_displayed); pose_it != last_pose_displayed - 1; ++pose_it)
      {
        const std::size_t i(std::distance(first_pose_displayed, pose_it));
        rvt_trajectory->publishCylinder(path.at(i), path.at(i + 1), colors.at(i), display_params.line_size);
      }
    }
    else // Display trajectory with cylinders for speed color mode
    {
      if (path.size() < 2)
        return "Invalid path because not enough points are passed in";

      if (path.size() != colors.size())
        return "Invalid path because " + std::to_string(path.size()) + " different from " +
               std::to_string(colors.size());

      for (auto pose_it(first_pose_displayed + 1); pose_it != last_pose_displayed; ++pose_it)
      {
        const std::size_t i(std::distance(first_pose_displayed, pose_it));
        // For speed color code only : to be in line with the speed modificator :
        // shift the color from the trajectory begining
        rvt_trajectory->publishCylinder(path.at(i - 1), path.at(i), colors.at(i), display_params.line_size);
      }
    }
  }
  else // Display trajectory with wires for all modes, the colors vector is modified accordingly before
  {
    std::vector<geometry_msgs::Point> a_points, b_points;
    for (auto it(path.begin()); it != --path.end(); ++it) // Skip last element
    {
      geometry_msgs::Point a, b;
      a.x = it->x();
      a.y = it->y();
      a.z = it->z();

      b.x = (it + 1)->x();
      b.y = (it + 1)->y();
      b.z = (it + 1)->z();

      a_points.emplace_back(a);
      b_points.emplace_back(b);
    }

    geometry_msgs::Vector3 scale;
    scale.x = display_params.line_size;
    scale.y = display_params.line_size;
    scale.z = display_params.line_size;

    rvt_trajectory->publishLines(a_points, b_points, colors, scale);
  }

  if (display_params.display_labels)
  {
    for (auto it(first_pose_displayed); it != last_pose_displayed; ++it)
    {
      // Offset the text so that it's not hidden inside the cylinder
      const double line_spacing(1.6); // How close the texts are together (space between lines)
      double z_text_offset(0);
      if (display_params.display_type == 0)
        z_text_offset += (display_params.line_size / line_spacing);

      geometry_msgs::Pose p(it->pose);
      p.position.z += z_text_offset;

      geometry_msgs::Vector3 scale;
      scale.x = display_params.label_size;
      scale.y = display_params.label_size;
      scale.z = display_params.label_size;

      switch (display_params.label_type)
      {
        case 0: // Pose number
          {
            rvt_trajectory->publishText(p,
                                        std::to_string(std::distance(trajectory.poses.begin(), it)),
                                        rviz_visual_tools::colors::WHITE,
                                        scale,
                                        false);
            break;
          }
        case 1: // Pose number within layer
          {
            AdditiveManufacturingPoseIt first_pose_of_layer =
            std::find_if(trajectory.poses.begin(), trajectory.poses.end(),
                         [ = ](const ram_msgs::AdditiveManufacturingPose & p)
            {
              return p.layer_index == it->layer_index;
            });

            rvt_trajectory->publishText(p,
                                        std::to_string(std::distance(first_pose_of_layer, it)),
                                        rviz_visual_tools::colors::WHITE,
                                        scale,
                                        false);
            break;
          }
        case 2: // Layer level
          {
            rvt_trajectory->publishText(p,
                                        std::to_string(it->layer_level),
                                        rviz_visual_tools::colors::WHITE,
                                        scale,
                                        false);
            break;
          }
        default: // 3 = Layer index
          {
            rvt_trajectory->publishText(p,
                                        std::to_string(it->layer_index),
                                        rviz_visual_tools::colors::WHITE,
                                        scale,
                                        false);
            break;
          }
      }
      p.position.z += display_params.label_size / line_spacing;

      if (display_params.label_pose_type)
      {
        if (it->entry_pose)
        {
          rvt_trajectory->publishText(p,
                                      "Entry pose",
                                      rviz_visual_tools::colors::WHITE,
                                      scale,
                                      false);
          p.position.z += display_params.label_size / line_spacing;
        }

        if (it->exit_pose)
        {
          rvt_trajectory->publishText(p,
                                      "Exit pose",
                                      rviz_visual_tools::colors::WHITE,
                                      scale,
                                      false);
          p.position.z += display_params.label_size / line_spacing;
        }

        if (it->polygon_start)
        {
          rvt_trajectory->publishText(p,
                                      "Polygon start",
                                      rviz_visual_tools::colors::WHITE,
                                      scale,
                                      false);
          p.position.z += display_params.label_size / line_spacing;
        }

        if (it->polygon_end)
        {
          rvt_trajectory->publishText(p,
                                      "Polygon end",
                                      rviz_visual_tools::colors::WHITE,
                                      scale,
                                      false);
          p.position.z += display_params.label_size / line_spacing;
        }
      }

      if (display_params.label_laser_power)
      {
        if (it == trajectory.poses.begin() || it == trajectory.poses.end() - 1 ||
            (it - 1)->params.laser_power != (it->params.laser_power) ||
            (it + 1)->params.laser_power != (it->params.laser_power))
        {
          rvt_trajectory->publishText(p,
                                      "Laser power = " + std::to_string((unsigned) it->params.laser_power) + " W",
                                      rviz_visual_tools::colors::CYAN,
                                      scale,
                                      false);
          p.position.z += display_params.label_size / line_spacing;
        }
      }

      if (display_params.label_feed_rate)
      {
        if (it == trajectory.poses.begin() || it == trajectory.poses.end() - 1 ||
            (it - 1)->params.feed_rate != (it->params.feed_rate) ||
            (it + 1)->params.feed_rate != (it->params.feed_rate))
        {
          rvt_trajectory->publishText(p,
                                      "Feed rate = " + toStringWithPrecision(it->params.feed_rate * 60) + " m/min",
                                      rviz_visual_tools::colors::PINK,
                                      scale,
                                      false);
          p.position.z += display_params.label_size / line_spacing;
        }
      }

      if (display_params.label_speed)
      {
        if (it == trajectory.poses.begin() || it == trajectory.poses.end() - 1 ||
            (it - 1)->params.speed != (it->params.speed) ||
            (it + 1)->params.speed != (it->params.speed))
        {
          rvt_trajectory->publishText(p,
                                      "Speed = " + toStringWithPrecision(it->params.speed * 60) + " m/min",
                                      rviz_visual_tools::colors::GREEN,
                                      scale,
                                      false);
          p.position.z += display_params.label_size / line_spacing;
        }
      }

      if (display_params.label_blend_radius)
      {
        if (it == trajectory.poses.begin() || it == trajectory.poses.end() - 1 ||
            (it - 1)->params.blend_radius != (it->params.blend_radius) ||
            (it + 1)->params.blend_radius != (it->params.blend_radius))
        {
          rvt_trajectory->publishText(p,
                                      "Blend radius = " + std::to_string((unsigned) it->params.blend_radius) + " %",
                                      rviz_visual_tools::colors::PURPLE,
                                      scale,
                                      false);
          p.position.z += display_params.label_size / line_spacing;
        }
      }

      if (display_params.label_approach_type)
      {
        if (it == trajectory.poses.begin() || it == trajectory.poses.end() - 1 ||
            (it - 1)->params.approach_type != (it->params.approach_type) ||
            (it + 1)->params.approach_type != (it->params.approach_type))
        {
          std::string approach_type("Stop/go");
          if (it->params.approach_type)
            approach_type = "Blend radius";
          rvt_trajectory->publishText(p,
                                      "Approach type = " + approach_type,
                                      rviz_visual_tools::colors::YELLOW,
                                      scale,
                                      false);
          p.position.z += display_params.label_size / line_spacing;
        }
      }

      if (display_params.label_movement_type)
      {
        if (it == trajectory.poses.begin() || it == trajectory.poses.end() - 1 ||
            (it - 1)->params.movement_type != (it->params.movement_type) ||
            (it + 1)->params.movement_type != (it->params.movement_type))
        {
          std::string movement_type("Joint");
          if (it->params.movement_type == 1)
            movement_type = "Linear";
          rvt_trajectory->publishText(p,
                                      "Movement type = " + movement_type,
                                      rviz_visual_tools::colors::MAGENTA,
                                      scale,
                                      false);
          p.position.z += display_params.label_size / line_spacing;
        }
      }
    }
  }
  rvt_trajectory->trigger();
  updateSelection();
  return "";
}

void trajectoryCallback(const ram_msgs::AdditiveManufacturingTrajectory::ConstPtr &msg)
{
  std::lock_guard<std::recursive_mutex> lock(trajectory_mutex);
  trajectory = *msg;
  if (!displayTrajectory().empty())
  {
    range_of_layers_params.display_range_of_layers = false;
    displayTrajectory();
  }
}

bool displayTrajectoryCallback(ram_display::DisplayTrajectory::Request &req,
                               ram_display::DisplayTrajectory::Response &res)
{
  std::lock_guard<std::recursive_mutex> lock(display_params_mutex);
  display_params = req;
  res.error = displayTrajectory();
  return true;
}

bool deleteTrajectoryCallback(ram_display::DeleteTrajectory::Request &,
                              ram_display::DeleteTrajectory::Response &)
{
  deleteTrajectory();
  return true;
}

bool updateSelection(const bool display)
{
  std::lock_guard<std::recursive_mutex> lock_1(display_params_mutex);
  std::lock_guard<std::recursive_mutex> lock_2(selection_params_mutex);

  rvt_selection->deleteAllMarkers();
  rvt_selection->trigger();

  if (!display)
    return true;

  std::vector<geometry_msgs::Point> points;
  for (auto ram_pose : selection.selected_poses)
    points.emplace_back(ram_pose.pose.position);

  geometry_msgs::Vector3 scale;
  scale.x = display_params.line_size * 1.8;
  scale.y = scale.x;
  scale.z = scale.x;

  std::vector<std_msgs::ColorRGBA> colors(points.size());
  std_msgs::ColorRGBA color;
  if (selection.temporary)
  {
    color.a = 0.5;
    color.r = 0.8;
    color.g = 0.8;
    color.b = 0.8;
  }
  else
  {
    color.a = 1.0;
    color.r = 1.0;
    color.g = 1.0;
    color.b = 1.0;
  }
  std::fill(colors.begin(), colors.end(), color);

  rvt_selection->publishSpheres(points, colors, scale);
  rvt_selection->trigger();
  return true;
}

bool updateSelectionCallback(ram_display::UpdateSelection::Request &req,
                             ram_display::UpdateSelection::Response &)
{
  std::lock_guard<std::recursive_mutex> lock_1(display_params_mutex);
  std::lock_guard<std::recursive_mutex> lock_2(selection_params_mutex);
  selection = req;
  return updateSelection();
}

bool updateMeshColorCallback(ram_display::UpdateMeshColor::Request &req,
                             ram_display::UpdateMeshColor::Response &)
{
  std::lock_guard<std::recursive_mutex> lock_1(display_params_mutex);

  display_params.mesh_color = req.color;
  displayMesh(trajectory.file,
              display_params.mesh_color,
              trajectory_frame);
  return true;
}

bool displayRangeOfLayersCallback(ram_display::DisplayRangeOfLayers::Request &req,
                                  ram_display::DisplayRangeOfLayers::Response &res)
{
  std::lock_guard<std::recursive_mutex> lock(range_of_layers_params_mutex);
  range_of_layers_params = req;
  res.error = displayTrajectory();
  return true;
}

int main(int argc,
         char **argv)
{
  ros::init(argc, argv, "ram_display");
  ros::NodeHandle nh;
  nh.param<std::string>("ram/trajectory_frame", trajectory_frame, "base");
  std::string trajectory_marker_topic, selection_marker_topic;
  nh.param<std::string>("ram/display/trajectory_marker_topic", trajectory_marker_topic, "/rvt_trajectory");
  nh.param<std::string>("ram/display/selection_marker_topic", selection_marker_topic, "/rvt_selection");
  get_tool_client = nh.serviceClient<ram_utils::GetTool>("ram/get_tool");

  // Default parameters
  display_params.display_type = 1;
  display_params.line_size = 0.001;
  display_params.mesh_color.r = 0.7;
  display_params.mesh_color.g = 0.7;
  display_params.mesh_color.b = 0.7;
  display_params.mesh_color.a = 1;

  mesh_pub = nh.advertise<visualization_msgs::Marker>("ram/display/mesh", 1, true);

  // Allow the action server to receive and send ROS messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  rvt_trajectory = std::make_shared<rviz_visual_tools::RvizVisualTools>(trajectory_frame, trajectory_marker_topic);
  rvt_selection = std::make_shared<rviz_visual_tools::RvizVisualTools>(trajectory_frame, selection_marker_topic);

  // Dynamic markers
  rvt_trajectory->enableFrameLocking();
  rvt_selection->enableFrameLocking();

  // Latch publish
  rvt_trajectory->loadMarkerPub(false, true);
  rvt_selection->loadMarkerPub(false, true);

  ros::Subscriber sub = nh.subscribe("ram/trajectory", 5, trajectoryCallback);

  ros::ServiceServer service_1 = nh.advertiseService("ram/display/add_trajectory", displayTrajectoryCallback);
  ros::ServiceServer service_2 = nh.advertiseService("ram/display/delete_trajectory", deleteTrajectoryCallback);
  ros::ServiceServer service_3 = nh.advertiseService("ram/display/update_selection", updateSelectionCallback);
  ros::ServiceServer service_4 = nh.advertiseService("ram/display/update_mesh_color", updateMeshColorCallback);
  ros::ServiceServer service_5 = nh.advertiseService("ram/display/display_range_of_layers",
                                 displayRangeOfLayersCallback);

  ros::waitForShutdown();
  spinner.stop();
  return 0;
}

