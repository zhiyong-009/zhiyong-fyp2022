#ifndef RAM_QT_GUIS_ENTRY_EXIT_STRATEGIES_HPP
#define RAM_QT_GUIS_ENTRY_EXIT_STRATEGIES_HPP

#include <eigen_conversions/eigen_msg.h>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QtConcurrent/QtConcurrentRun>
#include <ram_qt_guis/tree_button.hpp>
#include <ram_utils/EntryExitParameters.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>

namespace ram_qt_guis
{
class EntryExitStrategies : public rviz::Panel
{
Q_OBJECT

public:
  EntryExitStrategies(QWidget* parent = NULL);
  virtual ~EntryExitStrategies();

Q_SIGNALS:
  void enable(const bool);
  void displayMessageBox(const QString,
                         const QString,
                         const QString,
                         const QMessageBox::Icon);

private:
  void updateInternalParameters();
  void connectToService(ros::ServiceClient &client);
  void connectToServices();

protected Q_SLOTS:
  void sendEntryExitParameters();
  void displayMessageBoxHandler(const QString title,
                                const QString text,
                                const QString info = "",
                                const QMessageBox::Icon icon = QMessageBox::Icon::Information);

  void load(const rviz::Config& config);
  void save(rviz::Config config) const;

protected:
  QSpinBox *entry_number_of_poses_;
  QDoubleSpinBox *entry_angle_;
  QDoubleSpinBox *entry_distance_;
  QComboBox *entry_movement_type_;
  QComboBox *entry_approach_type_;
  QSpinBox *entry_blend_radius_;
  QDoubleSpinBox *entry_move_speed_;

  QSpinBox *exit_number_of_poses_;
  QDoubleSpinBox *exit_angle_;
  QDoubleSpinBox *exit_distance_;
  QComboBox *exit_movement_type_;
  QComboBox *exit_approach_type_;
  QSpinBox *exit_blend_radius_;
  QDoubleSpinBox *exit_move_speed_;

  QPushButton *entry_exit_button_;

  ros::NodeHandle nh_;
  ros::ServiceClient entry_parameters_client_;
  ros::ServiceClient exit_parameters_client_;

  ram_utils::EntryExitParameters entry_parameters_;
  ram_utils::EntryExitParameters exit_parameters_;

  double entry_conversion_factor_ = 1;
  double exit_conversion_factor_ = 1;
};

}

#endif
