#include <ram_qt_guis/simulate.hpp>

namespace ram_qt_guis
{

Simulate::Simulate(QWidget *parent) :
  rviz::Panel(parent)
{
  connect(this, &Simulate::enable, this, &Simulate::setEnabled);
  setObjectName("Simulate");
  setName(objectName());

  qRegisterMetaType<QMessageBox::Icon>();
  connect(this, &Simulate::displayMessageBox, this, &Simulate::displayMessageBoxHandler);

  QVBoxLayout *main_layout = new QVBoxLayout(this);
  simulate_button_ = new QPushButton("Simulate");
  main_layout->addWidget(simulate_button_);

  // Setup service clients
  simulate_trajectory_ = nh_.serviceClient<ram_trajectory::SimulateTrajectory>("ram_trajectory/simulate");

  // Check connection of clients
  connectToServices();
}

Simulate::~Simulate()
{
}

void Simulate::connectToService(ros::ServiceClient &client)
{
  Q_EMIT enable(false);

  while (nh_.ok())
  {
    if (client.waitForExistence(ros::Duration(2)))
    {
      ROS_INFO_STREAM("RViz panel " << getName().toStdString() << " connected to the service " << client.getService());
      break;
    }
    else
    {
      ROS_ERROR_STREAM(
        "RViz panel " << getName().toStdString() << " could not connect to ROS service: " << client.getService());
      ros::Duration(1).sleep();
    }
  }
}

void Simulate::connectToServices()
{
  Q_EMIT enable(false);
  connectToService(simulate_trajectory_);
  ROS_INFO_STREAM("RViz panel " << getName().toStdString() << " services connections have been made");
  Q_EMIT enable(true);
  connect(simulate_button_, &QPushButton::clicked, this, &Simulate::simulateButtonHandler);
}

void Simulate::simulateButtonHandler()
{
  QtConcurrent::run(this, &Simulate::simulateButton);
}

void Simulate::simulateButton()
{
  Q_EMIT enable(false);
  ram_trajectory::SimulateTrajectory srv;

  bool success = simulate_trajectory_.call(srv);
  Q_EMIT enable(true);

  if (!success)
  {
    Q_EMIT displayMessageBox("Simulate trajectory", "Could not call the simulate service", "", QMessageBox::Icon::Critical);
  }

  if (!srv.response.error.empty())
  {
    Q_EMIT displayMessageBox("Simulate trajectory", "Call to the simulate service failed with error:",
                             QString::fromStdString(srv.response.error), QMessageBox::Icon::Critical);
  }
}
void Simulate::displayMessageBoxHandler(const QString title,
                                        const QString text,
                                        const QString info,
                                        const QMessageBox::Icon icon)
{
  const bool old_state(isEnabled());
  setEnabled(false);
  QMessageBox msg_box;
  msg_box.setWindowTitle(title);
  msg_box.setText(text);
  msg_box.setInformativeText(info);
  msg_box.setIcon(icon);
  msg_box.setStandardButtons(QMessageBox::Ok);
  msg_box.exec();
  setEnabled(old_state);
}

void Simulate::load(const rviz::Config &config)
{
  rviz::Panel::load(config);
}

void Simulate::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ram_qt_guis::Simulate, rviz::Panel)

