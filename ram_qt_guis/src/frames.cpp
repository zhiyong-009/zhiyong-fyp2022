#include <ram_qt_guis/frames.hpp>

namespace ram_qt_guis
{
Frames::Frames(QWidget *parent) :
  rviz::Panel(parent)
{
  connect(this, &Frames::enable, this, &Frames::setEnabled);
  setObjectName("Frames");
  setName(objectName());

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
                               new TreeButton("Trajectory frame", tree_widget, category));

    QWidget *widget = new QWidget;
    QVBoxLayout *widget_layout(new QVBoxLayout);
    widget->setLayout(widget_layout);
    trajectory_frame_ = new Pose(QString::fromStdString("trajectory_frame"),
                                 QString::fromStdString(""));
    widget_layout->addWidget(trajectory_frame_);

    QTreeWidgetItem *container = new QTreeWidgetItem();
    container->setDisabled(true);
    category->addChild(container);
    tree_widget->setItemWidget(container, 0, widget);
  }

  {
    QTreeWidgetItem *category(new QTreeWidgetItem);
    tree_widget->addTopLevelItem(category);
    tree_widget->setItemWidget(category, 0,
                               new TreeButton("Start pose", tree_widget, category));

    QWidget *widget = new QWidget;
    QVBoxLayout *widget_layout(new QVBoxLayout);
    widget->setLayout(widget_layout);
    start_pose_ = new Pose(QString::fromStdString("start_pose"),
                           QString::fromStdString(""));
    widget_layout->addWidget(start_pose_);

    QTreeWidgetItem *container = new QTreeWidgetItem();
    container->setDisabled(true);
    category->addChild(container);
    tree_widget->setItemWidget(container, 0, widget);
  }

  {
    QTreeWidgetItem *category(new QTreeWidgetItem);
    tree_widget->addTopLevelItem(category);
    tree_widget->setItemWidget(category, 0,
                               new TreeButton("Tool orientation", tree_widget, category));

    QWidget *widget = new QWidget;
    QVBoxLayout *widget_layout(new QVBoxLayout);
    widget->setLayout(widget_layout);
    tool_ = new Pose(QString::fromStdString("tool"),
                     QString::fromStdString(""),
                     Pose::Mode::READ_WRITE_ORIENTATION);
    widget_layout->addWidget(tool_);

    QLabel *label(new QLabel("Display must be updated to see tool orientation changes when axis"
    " are displayed on the trajectory."));
    label->setWordWrap(true);
    widget_layout->addWidget(label);

    QTreeWidgetItem *container = new QTreeWidgetItem();
    container->setDisabled(true);
    category->addChild(container);
    tree_widget->setItemWidget(container, 0, widget);
  }

  qRegisterMetaType<QMessageBox::Icon>();
  connect(this, &Frames::displayMessageBox, this, &Frames::displayMessageBoxHandler);

  trajectory_frame_pub_ = nh_.advertise<geometry_msgs::Pose>("ram/trajectory_frame", 1);
  start_pose_pub_ = nh_.advertise<geometry_msgs::Pose>("ram/start_pose", 1);
  tool_pub_ = nh_.advertise<geometry_msgs::Pose>("ram/tool", 1);

  trajectory_frame_pose_.orientation.x = 0;
  trajectory_frame_pose_.orientation.y = 0;
  trajectory_frame_pose_.orientation.z = 0;
  trajectory_frame_pose_.orientation.w = 1;

  start_pose_pose_.orientation.x = 0;
  start_pose_pose_.orientation.y = 0;
  start_pose_pose_.orientation.z = 0;
  start_pose_pose_.orientation.w = 1;

  tool_pose_.orientation.x = 0;
  tool_pose_.orientation.y = 0;
  tool_pose_.orientation.z = 0;
  tool_pose_.orientation.w = 1;
}

Frames::~Frames()
{
}

void Frames::updateInternalParameters()
{
  tf::poseEigenToMsg(trajectory_frame_->pose(), trajectory_frame_pose_);
  tf::poseEigenToMsg(start_pose_->pose(), start_pose_pose_);
  tf::poseEigenToMsg(tool_->pose(), tool_pose_);
}

void Frames::sendTrajectoryFrameInformation()
{
  updateInternalParameters();
  if (trajectory_frame_pub_.getNumSubscribers() == 0)
  {
    Q_EMIT displayMessageBox(
      "Published message will be lost: ",
      QString::fromStdString(trajectory_frame_pub_.getTopic() + " has no subscriber"), "",
      QMessageBox::Icon::Critical);
  }
  trajectory_frame_pub_.publish(trajectory_frame_pose_);
  ros::spinOnce();
}

void Frames::sendStartPoseInformation()
{
  updateInternalParameters();
  if (start_pose_pub_.getNumSubscribers() == 0)
  {
    Q_EMIT displayMessageBox(
      "Published message will be lost: ",
      QString::fromStdString(start_pose_pub_.getTopic() + " has no subscriber"), "",
      QMessageBox::Icon::Critical);
  }
  start_pose_pub_.publish(start_pose_pose_);
  ros::spinOnce();
}

void Frames::sendToolInformation()
{
  updateInternalParameters();
  if (tool_pub_.getNumSubscribers() == 0)
  {
    Q_EMIT displayMessageBox(
      "Published message will be lost: ",
      QString::fromStdString(tool_pub_.getTopic() + " has no subscriber"), "",
      QMessageBox::Icon::Critical);
  }
  tool_pub_.publish(tool_pose_);
  ros::spinOnce();
}

void Frames::load(const rviz::Config &config)
{
  trajectory_frame_->load(config);
  start_pose_->load(config);
  tool_->load(config);
  rviz::Panel::load(config);
  QtConcurrent::run(this, &Frames::sendLoadedInformation);
}

void Frames::sendLoadedInformation()
{
  Q_EMIT enable(false);

  ros::Duration(1).sleep();
  while (nh_.ok())
  {
    if (trajectory_frame_pub_.getNumSubscribers() != 0 &&
        start_pose_pub_.getNumSubscribers() != 0 &&
        tool_pub_.getNumSubscribers() != 0)
      break;
    else
      ROS_WARN_STREAM_THROTTLE(1,
                               "RViz panel " << objectName().toStdString() + " did not connect to all the subscribers");
  }

  ROS_INFO_STREAM("RViz panel " << objectName().toStdString() << " subscribers connections have been made");

  sendTrajectoryFrameInformation();
  sendStartPoseInformation();
  sendToolInformation();

  connect(trajectory_frame_, &Pose::valueChanged, this, &Frames::sendTrajectoryFrameInformation);
  connect(start_pose_, &Pose::valueChanged, this, &Frames::sendStartPoseInformation);
  connect(tool_, &Pose::valueChanged, this, &Frames::sendToolInformation);

  // Connect with configChanged signal
  connect(trajectory_frame_, &Pose::valueChanged, this, &Frames::configChanged);
  connect(start_pose_, &Pose::valueChanged, this, &Frames::configChanged);
  connect(tool_, &Pose::valueChanged, this, &Frames::configChanged);

  Q_EMIT enable(true);
}

void Frames::save(rviz::Config config) const
{
  trajectory_frame_->save(config);
  start_pose_->save(config);
  tool_->save(config);
  rviz::Panel::save(config);
}

void Frames::displayMessageBoxHandler(const QString title,
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

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ram_qt_guis::Frames, rviz::Panel)
