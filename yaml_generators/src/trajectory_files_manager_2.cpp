#include <ram_utils/trajectory_files_manager_2.hpp>

namespace ram_utils
{

bool yamlFileToPoses(std::string yaml_file,
                     PosesTrajectory &poses)
{
  poses.clear();

  PosesLayer t_layer;
  PosesPolygon t_polygon;
  try
  {
    YAML::Node traj_node = YAML::LoadFile(yaml_file);

    for (const auto &node : traj_node)
    {
      for (const auto &layer_array : node)
      {
        YAML::Node key_layer = layer_array.first;
        if (!key_layer.IsScalar())
          continue;

        YAML::Node value_layer = layer_array.second;
        if (!value_layer.IsSequence())
          continue;

        if (key_layer.as<std::string>() != "layer")
          continue;

        for (const auto &layer : value_layer)
        {
          for (const auto &polygon : layer)
          {
            YAML::Node key_polygon = polygon.first;
            if (!key_polygon.IsScalar())
              continue;

            YAML::Node value_polygon = polygon.second;
            if (!value_polygon.IsSequence())
              continue;

            if (key_polygon.as<std::string>() != "polygon")
              continue;

            for (const auto &pose : value_polygon)
            {
              if (!pose.IsSequence())
                continue;

              Pose t_pose(Pose::Identity());
              std::vector<double> p = pose.as<std::vector<double>>();
              if (p.size() == 7)
              {
                // Warning: W, X, Y, Z initialization!
                t_pose.linear() = Eigen::Quaterniond(p[6], p[3], p[4], p[5]).toRotationMatrix();
              }
              else if (p.size() != 3)
                continue;

              t_pose.translation()[0] = p[0];
              t_pose.translation()[1] = p[1];
              t_pose.translation()[2] = p[2];

              t_polygon.push_back(t_pose);
            }
            if (!t_polygon.empty())
            {
              t_layer.push_back(t_polygon);
              t_polygon.clear();
            }
          }
        }
        if (!t_layer.empty())
        {
          poses.push_back(t_layer);
          t_layer.clear();
        }
      }
    }
  }
  catch (YAML::Exception &e)
  {
    ROS_ERROR_STREAM(e.what());
    return false;
  }

  if (poses.empty())
    return false;

  return true;
}

bool yamlFileToTrajectory(const std::string yaml_file,
                          ram_msgs::AdditiveManufacturingTrajectory &traj)
{
  traj.poses.clear();
  unsigned layer_number(0);
  try
  {
    YAML::Node traj_node = YAML::LoadFile(yaml_file);

    for (const auto &node : traj_node)
    {
      for (const auto &layer_array : node)
      {
        YAML::Node key_layer = layer_array.first;
        if (!key_layer.IsScalar())
          continue;

        YAML::Node value_layer = layer_array.second;
        if (!value_layer.IsSequence())
          continue;

        if (key_layer.as<std::string>() != "layer")
          continue;

        for (const auto &layer : value_layer)
        {
          for (const auto &polygon : layer)
          {
            YAML::Node key_polygon = polygon.first;
            if (!key_polygon.IsScalar())
              continue;

            YAML::Node value_polygon = polygon.second;
            if (!value_polygon.IsSequence())
              continue;

            if (key_polygon.as<std::string>() != "polygon")
              continue;

            unsigned i(0);
            for (const auto &pose : value_polygon)
            {
              if (!pose.IsSequence())
                continue;

              ram_msgs::AdditiveManufacturingPose ram_pose;
              std::vector<double> p = pose.as<std::vector<double>>();
              if (p.size() == 3)
              {
                ram_pose.pose.orientation.x = 0;
                ram_pose.pose.orientation.y = 0;
                ram_pose.pose.orientation.z = 0;
                ram_pose.pose.orientation.w = 1;
              }
              else if (p.size() == 7)
              {
                ram_pose.pose.orientation.x = p[3];
                ram_pose.pose.orientation.y = p[4];
                ram_pose.pose.orientation.z = p[5];
                ram_pose.pose.orientation.w = p[6];
              }
              else
                continue;

              ram_pose.layer_index = layer_number;
              ram_pose.layer_level = layer_number;
              ram_pose.entry_pose = false;
              ram_pose.exit_pose = false;
              ram_pose.unique_id = unique_id::toMsg(unique_id::fromRandom());
              ram_pose.pose.position.x = p[0];
              ram_pose.pose.position.y = p[1];
              ram_pose.pose.position.z = p[2];

              if (i == 0)
                ram_pose.polygon_start = true;

              traj.poses.push_back(ram_pose);
              ++i;
            }
            traj.poses.back().polygon_end = true;
          }
        }
        ++layer_number;
      }
    }
  }
  catch (YAML::Exception &e)
  {
    ROS_ERROR_STREAM(e.what());
    return false;
  }

  if (traj.poses.empty())
    return false;

  traj.file = yaml_file;
  traj.generated = ros::Time::now();
  traj.modified = traj.generated;
  traj.similar_layers = false;
  traj.generation_info = "From YAML (poses) file\n";
  return true;
}

}
