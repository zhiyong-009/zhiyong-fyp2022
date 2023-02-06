#ifndef INDUSTRIAL_ROBOT_ANGLE_CONVERSIONS_POSES_HPP
#define INDUSTRIAL_ROBOT_ANGLE_CONVERSIONS_POSES_HPP

#include <industrial_robot_angle_conversions/angles.hpp>

namespace industrial_robot_angle_conversions
{

const double deg_2_rad(M_PI / 180.0);
const double rad_2_deg(180.0 / M_PI);

void isometryToFanucPose(const Eigen::Isometry3d &pose,
                         Eigen::Vector3d &xyz_mm,
                         Eigen::Vector3d &wpr_deg);

void fanucPoseToIsometry(const Eigen::Vector3d &xyz_mm,
                         const Eigen::Vector3d &wpr_deg,
                         Eigen::Isometry3d &pose);

}

#endif
