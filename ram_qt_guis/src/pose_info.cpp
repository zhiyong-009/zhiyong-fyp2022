#include <ram_qt_guis/pose_info.hpp>

namespace ram_qt_guis
{

PoseInfo::PoseInfo()
{
  QVBoxLayout *layout(new QVBoxLayout);
  QWidget *scroll_widget = new QWidget;
  scroll_widget->setLayout(layout);
  QScrollArea *scroll_area = new QScrollArea;
  scroll_area->setWidget(scroll_widget);
  scroll_area->setWidgetResizable(true);
  scroll_area->setFrameShape(QFrame::NoFrame);
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  main_layout->addWidget(scroll_area);

  QTreeWidget *tree_widget(new QTreeWidget);
  layout->addWidget(tree_widget);
  tree_widget->setFrameStyle(QFrame::NoFrame);
  QPalette pal = tree_widget->palette();
  pal.setColor(QPalette::Base, Qt::transparent);
  tree_widget->setPalette(pal);
  tree_widget->setHeaderHidden(true);
  tree_widget->setIndentation(0);

  {
    QTreeWidgetItem *category(new QTreeWidgetItem);
    tree_widget->addTopLevelItem(category);
    tree_widget->setItemWidget(category, 0,
                               new TreeButton("Geometric pose", tree_widget, category));

    QWidget *widget = new QWidget;
    QVBoxLayout *widget_layout(new QVBoxLayout);
    widget->setLayout(widget_layout);
    pose_ = new Pose("", "", Pose::Mode::READ_ONLY, Pose::PoseType::FANUC);
    widget_layout->addWidget(pose_);

    QTreeWidgetItem *container = new QTreeWidgetItem();
    container->setDisabled(true);
    category->addChild(container);
    tree_widget->setItemWidget(container, 0, widget);
  }

  {
    QTreeWidgetItem *category(new QTreeWidgetItem);
    tree_widget->addTopLevelItem(category);
    tree_widget->setItemWidget(category, 0,
                               new TreeButton("Information", tree_widget, category));

    QWidget *widget = new QWidget;
    QGridLayout *widget_layout(new QGridLayout);
    widget->setLayout(widget_layout);
    pose_uuid_ = new QLabel;
    pose_uuid_->setFont(QFont("Hack"));
    pose_uuid_->setWordWrap(true);
    pose_uuid_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layer_level_ = new QLabel;
    layer_index_ = new QLabel;
    polygon_start_ = new QLabel;
    polygon_end_ = new QLabel;
    entry_pose_ = new QLabel;
    exit_pose_ = new QLabel;
    widget_layout->addWidget(new QLabel("Layer level"), 0, 0);
    widget_layout->addWidget(layer_level_, 0, 1);
    widget_layout->addWidget(new QLabel("Layer index"));
    widget_layout->addWidget(layer_index_);
    widget_layout->addWidget(new QLabel("Polygon start"));
    widget_layout->addWidget(polygon_start_);
    widget_layout->addWidget(new QLabel("Polygon end"));
    widget_layout->addWidget(polygon_end_);
    widget_layout->addWidget(new QLabel("Entry pose"));
    widget_layout->addWidget(entry_pose_);
    widget_layout->addWidget(new QLabel("Exit pose"));
    widget_layout->addWidget(exit_pose_);
    widget_layout->addWidget(new QLabel("UUID (hex)"));
    widget_layout->addWidget(pose_uuid_);

    QTreeWidgetItem *container = new QTreeWidgetItem();
    container->setDisabled(true);
    category->addChild(container);
    tree_widget->setItemWidget(container, 0, widget);
  }

  {
    QTreeWidgetItem *category(new QTreeWidgetItem);
    tree_widget->addTopLevelItem(category);
    tree_widget->setItemWidget(category, 0,
                               new TreeButton("Parameters", tree_widget, category));

    QWidget *widget = new QWidget;
    QGridLayout *widget_layout(new QGridLayout);

    widget->setLayout(widget_layout);
    movement_type_ = new QLabel;
    approach_type_ = new QLabel;
    blend_radius_ = new QLabel;
    speed_ = new QLabel;
    laser_power_ = new QLabel;
    feed_rate_ = new QLabel;
    widget_layout->addWidget(new QLabel("Movement type"), 0, 0);
    widget_layout->addWidget(movement_type_, 0, 1);
    widget_layout->addWidget(new QLabel("Approach type"));
    widget_layout->addWidget(approach_type_);
    widget_layout->addWidget(new QLabel("Blend radius"));
    widget_layout->addWidget(blend_radius_);
    widget_layout->addWidget(new QLabel("Speed"));
    widget_layout->addWidget(speed_);
    widget_layout->addWidget(new QLabel("Laser power"));
    widget_layout->addWidget(laser_power_);
    widget_layout->addWidget(new QLabel("Feed rate"));
    widget_layout->addWidget(feed_rate_);

    QTreeWidgetItem *container = new QTreeWidgetItem();
    container->setDisabled(true);
    category->addChild(container);
    tree_widget->setItemWidget(container, 0, widget);
  }

  QWidget *bottom(new QWidget);
  layout->addWidget(bottom);
  connect(this, &PoseInfo::enable, this, &PoseInfo::setEnabled);

  QVBoxLayout *bottom_layout = new QVBoxLayout;
  bottom->setLayout(bottom_layout);
  QHBoxLayout *pose_index(new QHBoxLayout);
  pose_index->addWidget(new QLabel("Pose index:"));
  pose_index_ = new QSpinBox;
  pose_index_->setRange(0, 0);
  pose_index->addWidget(pose_index_);
  bottom_layout->addLayout(pose_index);
  QHBoxLayout *range_buttons(new QHBoxLayout);
  first_ = new QPushButton("<<");
  back_ = new QPushButton("<");
  forward_ = new QPushButton(">");
  last_ = new QPushButton(">>");

  range_buttons->addWidget(first_);
  range_buttons->addWidget(back_);
  range_buttons->addWidget(forward_);
  range_buttons->addWidget(last_);
  bottom_layout ->addLayout(range_buttons);

  connect(first_, &QPushButton::clicked, this, [ = ]()
  {
    pose_index_->setValue(pose_index_->minimum());
  });
  connect(back_, &QPushButton::clicked, this, &PoseInfo::backButtonHandler);
  connect(forward_, &QPushButton::clicked, this, &PoseInfo::forwardButtonHandler);
  connect(last_, &QPushButton::clicked, this, [ = ]()
  {
    pose_index_->setValue(pose_index_->maximum());
  });

  connect(this, &PoseInfo::newTrajectorySize, this, &PoseInfo::newTrajectorySizeHandler);
  connect(pose_index_, qOverload<int>(&QSpinBox::valueChanged), this, &PoseInfo::getPoseInformation);
  Q_EMIT enable(false);

  trajectory_sub_ = nh_.subscribe("ram/trajectory", 1, &PoseInfo::trajectoryCallback, this);
  get_poses_from_trajectory_client_ =
    nh_.serviceClient<ram_modify_trajectory::GetPosesFromTrajectory>("ram/pose_selector/get_poses_from_trajectory");
}

void PoseInfo::load(const rviz::Config &config)
{
  rviz::Panel::load(config);
  pose_->load(config); // Recover mode
  const Eigen::Isometry3d identity(Eigen::Isometry3d::Identity());
  pose_->setPose(identity);
}

void PoseInfo::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  pose_->save(config);
}

void PoseInfo::getPoseInformation()
{
  std::lock_guard<std::mutex> lock(pose_params_mutex_);
  pose_params_.request.pose_index_list.clear();
  pose_params_.request.pose_index_list.emplace_back(pose_index_->value());

  bool success(get_poses_from_trajectory_client_.call(pose_params_));
  if (!success)
    return;
  if (pose_params_.response.poses.size() != 1)
    return;

  current_pose_layer_index_ = pose_params_.response.poses.at(0).layer_index;

  layer_level_->setText(QLocale().toString((int) pose_params_.response.poses.at(0).layer_level));
  layer_index_->setText(QLocale().toString((int) pose_params_.response.poses.at(0).layer_index));
  polygon_start_->setText((pose_params_.response.poses.at(0).polygon_start) ? "True" : "False");
  polygon_end_->setText((pose_params_.response.poses.at(0).polygon_end) ? "True" : "False");
  entry_pose_->setText((pose_params_.response.poses.at(0).entry_pose) ? "True" : "False");
  exit_pose_->setText((pose_params_.response.poses.at(0).exit_pose) ? "True" : "False");

  std::stringstream stream;
  for (auto & uuid : pose_params_.response.poses.at(0).unique_id.uuid)
    stream << std::hex << (unsigned) uuid;
  const std::string uuid_str = stream.str();
  pose_uuid_->setText(QString::fromStdString(uuid_str));

  // Geometric pose
  Eigen::Isometry3d pose;
  tf::poseMsgToEigen(pose_params_.response.poses.at(0).pose, pose);
  pose_->setPose(pose);

  // Others parameters
  QString movement_type_str;
  QString speed_unit_str = "";
  double speed_conversion_factor;
  switch (pose_params_.response.poses.at(0).params.movement_type)
  {
    case 0:
      movement_type_str = "Joint";
      speed_conversion_factor = 1.0;
      break;
    case 1:
      movement_type_str = "Linear";
      speed_unit_str = " meters/min";
      speed_conversion_factor = 60.0; // meters/sec --> meters/min
      break;
    default:
      movement_type_str = "";
      speed_conversion_factor = 0;
      break;
  }
  movement_type_->setText(movement_type_str);

  QString approach_type_str;
  switch (pose_params_.response.poses.at(0).params.approach_type)
  {
    case 0:
      approach_type_str = "Stop/go";
      break;
    case 1:
      approach_type_str = "Blend radius";
      break;
    default:
      approach_type_str = "-";
      break;
  }
  approach_type_->setText(approach_type_str);

  blend_radius_->setText(QLocale().toString(pose_params_.response.poses.at(0).params.blend_radius) + "%");
  speed_->setText(
    QLocale().toString(pose_params_.response.poses.at(0).params.speed * speed_conversion_factor) + speed_unit_str);
  laser_power_->setText(QLocale().toString(pose_params_.response.poses.at(0).params.laser_power) + " W");
  feed_rate_->setText(QLocale().toString(pose_params_.response.poses.at(0).params.feed_rate * 60.0)
    + " meters/min"); // meters/sec --> meters/min
}

void PoseInfo::backButtonHandler()
{
  if (pose_index_->value() == 0 || current_pose_layer_index_ == 0)
  {
    pose_index_->setValue(0);
    return;
  }

  pose_params_.request.pose_index_list.resize(1);
  pose_params_.request.pose_index_list.at(0) = pose_index_->value();
  while (pose_params_.request.pose_index_list.at(0) != 0)
  {
    --pose_params_.request.pose_index_list.at(0);
    bool success(get_poses_from_trajectory_client_.call(pose_params_));
    if (!success)
      return;
    if (pose_params_.response.poses.size() != 1)
      return;

    if (pose_params_.response.poses.at(0).layer_index == current_pose_layer_index_ - 1)
      break;
  }

  pose_index_->setValue(pose_params_.request.pose_index_list.at(0) + 1);
}

void PoseInfo::forwardButtonHandler()
{
  pose_params_.request.pose_index_list.resize(1);
  pose_params_.request.pose_index_list.at(0) = pose_index_->value() + 1;
  while (1)
  {
    bool success(get_poses_from_trajectory_client_.call(pose_params_));
    if (!success)
      return;
    if (pose_params_.response.poses.size() != 1)
      return;

    if (pose_params_.response.poses.at(0).layer_index == current_pose_layer_index_ + 1)
      break;

    ++pose_params_.request.pose_index_list.at(0);
  }

  pose_index_->setValue(pose_params_.request.pose_index_list.at(0) - 1);
}

void PoseInfo::trajectoryCallback(const ram_msgs::AdditiveManufacturingTrajectoryConstPtr &msg)
{
  Q_EMIT newTrajectorySize(msg->poses.size());
}

void PoseInfo::newTrajectorySizeHandler(const unsigned size)
{
  if (size == 0)
    Q_EMIT enable(false);
  else
    Q_EMIT enable(true);

  pose_index_->setRange(0, size - 1);
  pose_index_->valueChanged(pose_index_->value());
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ram_qt_guis::PoseInfo, rviz::Panel)
