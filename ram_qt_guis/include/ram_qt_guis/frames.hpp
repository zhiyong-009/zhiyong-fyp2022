#ifndef RAM_QT_GUIS_TRAJECTORY_FRAMES_HPP
#define RAM_QT_GUIS_TRAJECTORY_FRAMES_HPP

#include <eigen_conversions/eigen_msg.h>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QtConcurrent/QtConcurrentRun>
#include <ram_qt_guis/pose.hpp>
#include <ram_qt_guis/tree_button.hpp>
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>

namespace ram_qt_guis
{
class Frames : public rviz::Panel
{
Q_OBJECT
  public:
  Frames(QWidget* parent = NULL);
  virtual ~Frames();

Q_SIGNALS:
  void enable(const bool);
  void displayMessageBox(const QString,
                         const QString,
                         const QString,
                         const QMessageBox::Icon);

private:
  void updateInternalParameters();

protected Q_SLOTS:
  void sendInformationButtonHandler();
  void sendTrajectoryFrameInformation();
  void sendStartPoseInformation();
  void sendToolInformation();

  void load(const rviz::Config& config);
  void sendLoadedInformation();
  void save(rviz::Config config) const;

  void displayMessageBoxHandler(const QString title,
                                const QString text,
                                const QString info = "",
                                const QMessageBox::Icon icon = QMessageBox::Icon::Information);

protected:
  Pose *trajectory_frame_;
  Pose *start_pose_;
  Pose *tool_;

  ros::NodeHandle nh_;

  ros::Publisher trajectory_frame_pub_;
  geometry_msgs::Pose trajectory_frame_pose_;
  ros::Publisher start_pose_pub_;
  geometry_msgs::Pose start_pose_pose_;
  ros::Publisher tool_pub_;
  geometry_msgs::Pose tool_pose_;
};

}

#endif
