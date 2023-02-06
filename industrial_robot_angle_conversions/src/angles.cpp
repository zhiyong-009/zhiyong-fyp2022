#include <industrial_robot_angle_conversions/angles.hpp>

namespace industrial_robot_angle_conversions
{

Eigen::Quaterniond RPYtoQuaternion(const Eigen::Vector3d &rpy)
{
  using Eigen::AngleAxisd;
  using Eigen::Vector3d;
  // rpy has to be in radians
  return (AngleAxisd(rpy[0], Vector3d::UnitX())
          *  AngleAxisd(rpy[1], Vector3d::UnitY())
          *  AngleAxisd(rpy[2], Vector3d::UnitZ()));
}

Eigen::Quaterniond fanucWPRtoQuaternion(const Eigen::Vector3d &wpr)
{
  using Eigen::AngleAxisd;
  using Eigen::Vector3d;
  // wpr has to be in radians
  return (AngleAxisd(wpr[2], Vector3d::UnitZ())
          *  AngleAxisd(wpr[1], Vector3d::UnitY())
          *  AngleAxisd(wpr[0], Vector3d::UnitX()));
}

Eigen::Vector3d quaternionToRPY(const Eigen::Quaterniond &rot)
{
  Eigen::Vector3d rpy(rot.matrix().eulerAngles(0, 1, 2));
  return rpy; // In radians
}

Eigen::Vector3d quaternionToFanucWPR(const Eigen::Quaterniond &rot)
{
  Eigen::Vector3d wpr(rot.matrix().eulerAngles(2, 1, 0));
  std::swap(wpr[0], wpr[2]);
  return wpr; // In radians
}

}
