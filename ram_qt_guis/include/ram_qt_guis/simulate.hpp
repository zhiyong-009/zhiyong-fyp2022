#ifndef RAM_QT_GUIS_SIMULATE_HPP
#define RAM_QT_GUIS_SIMULATE_HPP

#include <QMessageBox>
#include <QPushButton>
#include <QtConcurrent/QtConcurrentRun>
#include <QVBoxLayout>
#include <ram_trajectory/SimulateTrajectory.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>

namespace ram_qt_guis
{
class Simulate : public rviz::Panel
{
  Q_OBJECT
public:
  Simulate(QWidget *parent = NULL);
  virtual ~Simulate();

Q_SIGNALS:
  void enable(const bool);
  void displayMessageBox(const QString,
                         const QString,
                         const QString,
                         const QMessageBox::Icon);
protected Q_SLOTS:
  void simulateButtonHandler();
  void load(const rviz::Config &config);
  void save(rviz::Config config) const;
  void displayMessageBoxHandler(const QString title,
                                const QString text,
                                const QString info = "",
                                const QMessageBox::Icon icon = QMessageBox::Icon::Information);

protected:
  void connectToService(ros::ServiceClient &client);
  void connectToServices();
  void simulateButton();

  ros::NodeHandle nh_;
  ros::ServiceClient simulate_trajectory_;

  QPushButton *simulate_button_;
};

}

#endif
