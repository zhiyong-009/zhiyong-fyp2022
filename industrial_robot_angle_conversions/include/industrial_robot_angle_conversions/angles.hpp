#ifndef INDUSTRIAL_ROBOT_ANGLE_CONVERSIONS_ANGLES_HPP
#define INDUSTRIAL_ROBOT_ANGLE_CONVERSIONS_ANGLES_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace industrial_robot_angle_conversions
{
// rpy has to be in radians
Eigen::Quaterniond RPYtoQuaternion(const Eigen::Vector3d &rpy);

// wpr has to be in radians
Eigen::Quaterniond fanucWPRtoQuaternion(const Eigen::Vector3d &wpr);

Eigen::Vector3d quaternionToRPY(const Eigen::Quaterniond &rot);

Eigen::Vector3d quaternionToFanucWPR(const Eigen::Quaterniond &rot);

}

#endif
