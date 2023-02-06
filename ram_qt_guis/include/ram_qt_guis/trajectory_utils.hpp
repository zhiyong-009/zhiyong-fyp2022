#ifndef RAM_QT_GUIS_TRAJECTORY_UTILS_HPP
#define RAM_QT_GUIS_TRAJECTORY_UTILS_HPP

#include <eigen_conversions/eigen_msg.h>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QtConcurrent/QtConcurrentRun>
#include <QVBoxLayout>
#include <ram_utils/BufferParams.h>
#include <ram_utils/ExportTrajectory.h>
#include <ram_utils/ImportTrajectory.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>

namespace ram_qt_guis
{
class TrajectoryUtils : public rviz::Panel
{
Q_OBJECT
  public:
  TrajectoryUtils(QWidget* parent = NULL);
  virtual ~TrajectoryUtils();
  void connectToService(ros::ServiceClient &client);
  void connectToServices();

Q_SIGNALS:
  void enable(const bool);
  void displayMessageBox(const QString,
                         const QString,
                         const QString,
                         const QMessageBox::Icon);

protected Q_SLOTS:
  void backButtonHandler();
  void forwardButtonHandler();
  void sendButton();
  void browseFilesToExportTrajectory();
  void browseFilesToImportTrajectory();
  void exportTrajectory();
  void importTrajectory();

  void load(const rviz::Config& config);
  void save(rviz::Config config) const;

  void displayMessageBoxHandler(const QString title,
                                const QString text,
                                const QString info = "",
                                const QMessageBox::Icon icon = QMessageBox::Icon::Information);

protected:
  QPushButton *back_button_;
  QPushButton *forward_button_;

  ros::NodeHandle nh_;
  ram_utils::BufferParams params_; //Back/forward button
  ram_utils::ExportTrajectory export_filename_;
  ram_utils::ImportTrajectory import_filename_;

  ros::ServiceClient trajectory_buffer_client_;
  ros::ServiceClient export_trajectory_client_;
  ros::ServiceClient import_trajectory_client_;
};

}

#endif
