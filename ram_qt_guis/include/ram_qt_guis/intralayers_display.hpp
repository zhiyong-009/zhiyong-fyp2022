#ifndef RAM_QT_GUIS_INTRALAYERS_DISPLAY_HPP
#define RAM_QT_GUIS_INTRALAYERS_DISPLAY_HPP

#include <QCheckBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QStackedWidget>
#include <QtConcurrent/QtConcurrentRun>
#include <ram_display/DisplayRangeOfLayers.h>
#include <ram_msgs/AdditiveManufacturingTrajectoryInfo.h>
#include <ram_qt_guis/ctk/ctkRangeSlider.hpp>
#include <ram_utils/GetNumberOfLayersLevels.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>

namespace ram_qt_guis
{
class IntralayersDisplay : public rviz::Panel
{

  Q_OBJECT

public:
  IntralayersDisplay(QWidget *parent = NULL);
  virtual ~IntralayersDisplay();

Q_SIGNALS:
  void enable(const bool);
  void layerRangeChanged(int numbers_of_layers);
  void displayMessageBox(const QString,
                         const QString,
                         const QString,
                         const QMessageBox::Icon);

protected Q_SLOTS:
  void updateDisplay();
  void enableLayersDisplay(bool state);
  void displayRangeOfLayers();
  void sendDisplayInformation();
  void changeLayerRange(int number_of_layers);
  void tweakFirstLayer();
  void tweakLastLayer();
  void load(const rviz::Config &config);
  void save(rviz::Config config) const;
  void displayMessageBoxHandler(const QString title,
                                const QString text,
                                const QString info = "",
                                const QMessageBox::Icon icon = QMessageBox::Icon::Information);
  void setHorizontalLayout();
  void setVerticalLayout();

private:
  void connectToService(ros::ServiceClient &client);
  void connectToServicesAndPublishers();
  void checkForPublishers();
  void updateInternalParameters();
  void callBackTrajectoryInformation(const ram_msgs::AdditiveManufacturingTrajectoryInfoConstPtr &msg);
  void changeLayout();

private:
  ros::Subscriber subcriber_;
  ros::NodeHandle nh_;
  ros::ServiceClient display_client_;
  ram_display::DisplayRangeOfLayers params_;
  QStackedWidget *stacked_widget_;
  QPushButton *horizontal_;
  QPushButton *vertical_;
  QWidget *first_layer_widget_;
  QWidget *last_layer_widget_;
  QWidget *range_widget_;
  ctkRangeSlider *range_slider_;
  QCheckBox *range_of_layers_check_box_;
  QSpinBox *first_layer_spin_box_;
  QSpinBox *last_layer_spin_box_;
  QString layout_;
};
}
#endif
