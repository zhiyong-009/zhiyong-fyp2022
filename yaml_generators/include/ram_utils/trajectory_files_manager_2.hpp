#ifndef RAM_UTILS_TRAJECTORY_FILES_MANAGER_2_HPP
#define RAM_UTILS_TRAJECTORY_FILES_MANAGER_2_HPP


#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_utils/trajectory_files_manager_1.hpp>
#include <ros/ros.h>
#include <string>
#include <unique_id/unique_id.h>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace ram_utils
{
bool yamlFileToPoses(std::string yaml_file,
                     PosesTrajectory &poses);

bool yamlFileToTrajectory(const std::string yaml_file,
                          ram_msgs::AdditiveManufacturingTrajectory &traj);
}

#endif

