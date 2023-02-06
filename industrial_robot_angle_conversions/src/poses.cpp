#include <industrial_robot_angle_conversions/poses.hpp>

namespace industrial_robot_angle_conversions
{

void isometryToFanucPose(const Eigen::Isometry3d &pose,
                         Eigen::Vector3d &xyz_mm,
                         Eigen::Vector3d &wpr_deg)
{
  xyz_mm.x() = pose.translation().x() * 1e3;
  xyz_mm.y() = pose.translation().y() * 1e3;
  xyz_mm.z() = pose.translation().z() * 1e3;

  Eigen::Quaterniond q(pose.linear());
  wpr_deg = quaternionToFanucWPR(q);
  wpr_deg *= rad_2_deg;
}

void fanucPoseToIsometry(const Eigen::Vector3d &xyz_mm,
                         const Eigen::Vector3d &wpr_deg,
                         Eigen::Isometry3d &pose)
{
  pose = Eigen::Isometry3d::Identity();
  pose.translation().x() = xyz_mm.x() / 1e3;
  pose.translation().y() = xyz_mm.y() / 1e3;
  pose.translation().z() = xyz_mm.z() / 1e3;
  pose.linear() = fanucWPRtoQuaternion(wpr_deg * deg_2_rad).matrix();
}

}
