#include <ram_qt_guis/traj_info.hpp>

namespace ram_qt_guis
{

TrajInfo::TrajInfo(QWidget *parent) :
    rviz::Panel(parent)
{
  connect(this, &TrajInfo::enable, this, &TrajInfo::setEnabled);
  setObjectName("Trajectory info");
  setName(objectName());

  file_ = new QLabel;
  file_->setWordWrap(true);
  generated_ = new QLabel;
  modified_ = new QLabel;
  similar_layers_ = new QLabel;
  number_of_layers_levels_ = new QLabel;
  number_of_layers_indices_ = new QLabel;
  number_of_polygons_ = new QLabel;
  number_of_poses_ = new QLabel;
  trajectory_length_ = new QLabel;
  execution_time_ = new QLabel;
  deposit_time_ = new QLabel;
  wire_length_ = new QLabel;
  information_ = new QLabel;
  information_->setWordWrap(true);

  QVBoxLayout *scroll_widget_layout = new QVBoxLayout();
  QWidget *scroll_widget = new QWidget;
  scroll_widget->setLayout(scroll_widget_layout);
  QScrollArea *scroll_area = new QScrollArea;
  scroll_area->setWidget(scroll_widget);
  scroll_area->setWidgetResizable(true);
  scroll_area->setFrameShape(QFrame::NoFrame);
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  main_layout->addWidget(scroll_area);

  QGridLayout *grid_layout = new QGridLayout;
  scroll_widget_layout->addLayout(grid_layout);

  grid_layout->addWidget(new QLabel("File:"), 0, 0);
  grid_layout->addWidget(file_, 0, 1);

  grid_layout->addWidget(new QLabel("Generated at:"));
  grid_layout->addWidget(generated_);

  grid_layout->addWidget(new QLabel("Modified at:"));
  grid_layout->addWidget(modified_);

  grid_layout->addWidget(new QLabel("Similar layers:"));
  grid_layout->addWidget(similar_layers_);

  grid_layout->addWidget(new QLabel("Number of layers levels:"));
  grid_layout->addWidget(number_of_layers_levels_);

  grid_layout->addWidget(new QLabel("Number of layers indices:"));
  grid_layout->addWidget(number_of_layers_indices_);

  grid_layout->addWidget(new QLabel("Number of polygons:"));
  grid_layout->addWidget(number_of_polygons_);

  grid_layout->addWidget(new QLabel("Number of poses:"));
  grid_layout->addWidget(number_of_poses_);

  grid_layout->addWidget(new QLabel("Trajectory length:"));
  grid_layout->addWidget(trajectory_length_);

  grid_layout->addWidget(new QLabel("Execution time:"));
  grid_layout->addWidget(execution_time_);

  grid_layout->addWidget(new QLabel("Deposit time:"));
  grid_layout->addWidget(deposit_time_);

  grid_layout->addWidget(new QLabel("Wire length:"));
  grid_layout->addWidget(wire_length_);

  grid_layout->addWidget(new QLabel("Informations:"));
  grid_layout->addWidget(information_);

  sub_ = nh_.subscribe("ram/information/trajectory", 1, &TrajInfo::callback, this);

  // Check if there is any publisher
  QtConcurrent::run(this, &TrajInfo::checkForPublishers);
}

TrajInfo::~TrajInfo()
{
}

void TrajInfo::checkForPublishers()
{
  Q_EMIT enable(false);
  ros::Duration(0.5).sleep();

  while (nh_.ok())
  {
    if (sub_.getNumPublishers() != 0)
    {
      ROS_INFO_STREAM(
          "RViz panel " << getName().toStdString() << " topic " << sub_.getTopic() << " has at least one publisher");
      break;
    }
    else
    {
      ROS_ERROR_STREAM(
          "RViz panel " << getName().toStdString() << " topic " << sub_.getTopic() << " has zero publishers!");
      ros::Duration(1).sleep();
    }
  }

  Q_EMIT enable(true);
}

void TrajInfo::callback(const ram_msgs::AdditiveManufacturingTrajectoryInfoConstPtr &msg)
{
  std::lock_guard<std::recursive_mutex> lock(msg_mutex_);
  msg_ = *msg;
  Q_EMIT updateGUIFromParameters();
}

void TrajInfo::updateGUIFromParameters()
{
  std::lock_guard<std::recursive_mutex> lock(msg_mutex_);
  file_->setText(QString::fromStdString(msg_.file));
  QDateTime time;
  time.setTime_t(uint(msg_.generated.toSec()));
  generated_->setText(time.toString("dd/MM/yyyy - hh:mm:ss"));
  time.setTime_t(uint(msg_.modified.toSec()));
  modified_->setText(time.toString("dd/MM/yyyy - hh:mm:ss"));
  similar_layers_->setText(msg_.similar_layers ? "Yes" : "No");
  number_of_layers_levels_->setText(QLocale().toString((int) msg_.number_of_layers_levels));
  number_of_layers_indices_->setText(QLocale().toString((int) msg_.number_of_layers_indices));
  number_of_polygons_->setText(QLocale().toString((int) msg_.number_of_polygons));
  number_of_poses_->setText(QLocale().toString((int) msg_.number_of_poses));
  trajectory_length_->setText(QLocale().toString(msg_.trajectory_length, 'f', 3) + " meters");
  execution_time_->setText(QDateTime::fromMSecsSinceEpoch(msg_.execution_time * 1000.0).toUTC().toString("hh:mm:ss"));
  deposit_time_->setText(QDateTime::fromMSecsSinceEpoch(msg_.deposit_time * 1000.0).toUTC().toString("hh:mm:ss"));
  wire_length_->setText(QLocale().toString(msg_.wire_length, 'f', 1) + " meters");
  information_->setText(QString::fromStdString(msg_.generation_info));
}

void TrajInfo::load(const rviz::Config &config)
{
  rviz::Panel::load(config);
}

void TrajInfo::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(ram_qt_guis::TrajInfo, rviz::Panel)
