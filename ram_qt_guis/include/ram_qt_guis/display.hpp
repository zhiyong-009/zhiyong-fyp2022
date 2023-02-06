#ifndef RAM_QT_GUIS_DISPLAY_HPP
#define RAM_QT_GUIS_DISPLAY_HPP

#include <eigen_conversions/eigen_msg.h>
#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QRadioButton>
#include <QScrollArea>
#include <QtConcurrent/QtConcurrentRun>
#include <QTreeWidget>
#include <ram_display/DeleteTrajectory.h>
#include <ram_display/DisplayTrajectory.h>
#include <ram_display/UpdateMeshColor.h>
#include <ram_qt_guis/tree_button.hpp>
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>

namespace ram_qt_guis
{
class Display : public rviz::Panel
{
Q_OBJECT
  public:
  Display(QWidget* parent = NULL);
  virtual ~Display();

Q_SIGNALS:
  void enable(const bool);
  void displayMessageBox(const QString,
                         const QString,
                         const QString,
                         const QMessageBox::Icon);

private:
  void connectToService(ros::ServiceClient &client);
  void connectToServices();
  void updateInternalParameters();

protected Q_SLOTS:
  void pickColor();
  void sendDisplayInformationButtonHandler();
  void sendDisplayInformation();
  void sendDeleteInformationButtonHandler();
  void sendDeleteInformation();

  void load(const rviz::Config& config);
  void sendLoadedInformation();
  void save(rviz::Config config) const;

  void displayMessageBoxHandler(const QString title,
                                const QString text,
                                const QString info = "",
                                const QMessageBox::Icon icon = QMessageBox::Icon::Information);
protected:
  QComboBox *display_type_;
  QDoubleSpinBox *line_size_;
  QComboBox *color_by_;
  QCheckBox *display_axis_;
  QDoubleSpinBox *axis_size_;
  QCheckBox *display_labels_;
  QDoubleSpinBox *label_size_;
  QPushButton *labels_displayed_;
  QPushButton *mesh_color_;
  QPushButton *display_button_;
  QPushButton *delete_button_;
  QCheckBox *change_alpha_;
  QSpinBox *entry_pose_alpha_;
  QSpinBox *exit_pose_alpha_;
  QSpinBox *printing_pose_alpha_;

  ros::NodeHandle nh_;
  ros::ServiceClient display_client_;
  ros::ServiceClient delete_client_;
  ros::ServiceClient update_mesh_color_client_;
  ram_display::DisplayTrajectory params_;
};

}

#endif
