#ifndef RAM_QT_GUIS_POSE_INFO_HPP
#define RAM_QT_GUIS_POSE_INFO_HPP

#include <eigen_conversions/eigen_msg.h>
#include <mutex>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <ram_modify_trajectory/GetPosesFromTrajectory.h>
#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_qt_guis/pose.hpp>
#include <ram_qt_guis/tree_button.hpp>
#include <ros/ros.h>
#include <rviz/panel.h>

namespace ram_qt_guis
{
class PoseInfo : public rviz::Panel
{
  Q_OBJECT

public:
  PoseInfo();

Q_SIGNALS:
  void enable(bool);
  void newTrajectorySize(const unsigned);

protected Q_SLOTS:
  void load(const rviz::Config &config);
  void save(rviz::Config config) const;
  void backButtonHandler();
  void forwardButtonHandler();
  void newTrajectorySizeHandler(const unsigned size);
  void getPoseInformation();

private:
  void trajectoryCallback(const ram_msgs::AdditiveManufacturingTrajectoryConstPtr &msg);
  void connectToService();

  ros::NodeHandle nh_;
  ros::Subscriber trajectory_sub_;
  ros::ServiceClient get_poses_from_trajectory_client_;
  std::mutex pose_params_mutex_;
  ram_modify_trajectory::GetPosesFromTrajectory pose_params_;
  unsigned current_pose_layer_index_ = 0;

  Pose *pose_;

  QLabel *pose_uuid_;
  QLabel *layer_level_;
  QLabel *layer_index_;
  QLabel *polygon_start_;
  QLabel *polygon_end_;
  QLabel *entry_pose_;
  QLabel *exit_pose_;

  QLabel *movement_type_;
  QLabel *approach_type_;
  QLabel *blend_radius_;
  QLabel *speed_;
  QLabel *laser_power_;
  QLabel *feed_rate_;

  QSpinBox *pose_index_;
  QPushButton *first_;
  QPushButton *back_;
  QPushButton *forward_;
  QPushButton *last_;
};

}

#endif
