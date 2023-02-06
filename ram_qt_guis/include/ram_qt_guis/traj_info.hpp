#ifndef RAM_QT_GUIS_TRAJ_INFO_HPP
#define RAM_QT_GUIS_TRAJ_INFO_HPP

#include <mutex>
#include <QDateTime>
#include <QHBoxLayout>
#include <QLabel>
#include <QScrollArea>
#include <QtConcurrent/QtConcurrentRun>
#include <QVBoxLayout>
#include <ram_msgs/AdditiveManufacturingTrajectoryInfo.h>
#include <ros/ros.h>
#include <rviz/panel.h>

namespace ram_qt_guis
{
class TrajInfo : public rviz::Panel
{
Q_OBJECT
  public:
  TrajInfo(QWidget* parent = NULL);
  virtual ~TrajInfo();

Q_SIGNALS:
  void enable(const bool);

private:
  void checkForPublishers();
  void callback(const ram_msgs::AdditiveManufacturingTrajectoryInfoConstPtr& msg);

protected Q_SLOTS:
  void updateGUIFromParameters();
  void load(const rviz::Config& config);
  void save(rviz::Config config) const;

protected:
  QLabel *file_;
  QLabel *generated_;
  QLabel *modified_;
  QLabel *similar_layers_;
  QLabel *number_of_layers_levels_;
  QLabel *number_of_layers_indices_;
  QLabel *number_of_polygons_;
  QLabel *number_of_poses_;
  QLabel *trajectory_length_;
  QLabel *execution_time_;
  QLabel *deposit_time_;
  QLabel *wire_length_;
  QLabel *information_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::recursive_mutex msg_mutex_;
  ram_msgs::AdditiveManufacturingTrajectoryInfo msg_;
};

}

#endif
