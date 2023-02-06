#include <ram_qt_guis/trajectory_utils.hpp>

namespace ram_qt_guis
{
TrajectoryUtils::TrajectoryUtils(QWidget* parent) :
        rviz::Panel(parent)
{
  connect(this, &TrajectoryUtils::enable, this, &TrajectoryUtils::setEnabled);
  setObjectName("Trajectory utilities");
  setName(objectName());

  QSizePolicy policy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  back_button_ = new QPushButton("Back");
  back_button_->setSizePolicy(policy);
  forward_button_ = new QPushButton("Forward");
  forward_button_->setSizePolicy(policy);

  // Back-forward layout
  QHBoxLayout* back_forward_layout = new QHBoxLayout;
  back_forward_layout->addWidget(back_button_);
  back_forward_layout->addWidget(forward_button_);

  // Import-Export layout
  QHBoxLayout *import_export_file_layout = new QHBoxLayout;
  QPushButton *import_file_explorer = new QPushButton;
  import_file_explorer->setText("Import");
  QPushButton *export_file_explorer = new QPushButton;
  export_file_explorer->setText("Export");
  import_export_file_layout->addWidget(import_file_explorer);
  import_export_file_layout->addWidget(export_file_explorer);

  // Main Layout
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->addLayout(back_forward_layout);
  main_layout->addStretch(1);
  main_layout->addLayout(import_export_file_layout);

  //Connect buttons
  connect(back_button_, &QPushButton::clicked, this, &TrajectoryUtils::backButtonHandler);
  connect(forward_button_, &QPushButton::clicked, this, &TrajectoryUtils::forwardButtonHandler);
  connect(export_file_explorer, &QPushButton::released, this, &TrajectoryUtils::browseFilesToExportTrajectory);
  connect(import_file_explorer, &QPushButton::released, this, &TrajectoryUtils::browseFilesToImportTrajectory);

  qRegisterMetaType<QMessageBox::Icon>();
  connect(this, &TrajectoryUtils::displayMessageBox, this, &TrajectoryUtils::displayMessageBoxHandler);

  trajectory_buffer_client_ = nh_.serviceClient<ram_utils::BufferParams>("ram/buffer/get_trajectory");

  export_trajectory_client_ = nh_.serviceClient<ram_utils::ExportTrajectory>(
      "ram/buffer/export_trajectory");
  import_trajectory_client_ = nh_.serviceClient<ram_utils::ImportTrajectory>(
      "ram/buffer/import_trajectory");

  // Check connection of client
  QtConcurrent::run(this, &TrajectoryUtils::connectToServices);
}

TrajectoryUtils::~TrajectoryUtils()
{
}

void TrajectoryUtils::connectToService(ros::ServiceClient &client)
{
  while (nh_.ok())
  {
    if (client.waitForExistence(ros::Duration(2)))
    {
      ROS_INFO_STREAM(
                      "RViz panel " << getName().toStdString() << " connected to the service " << client.getService());
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

void TrajectoryUtils::connectToServices()
{
  Q_EMIT enable(false);
  // Back/forward button service
  connectToService(trajectory_buffer_client_);
  // Import/export trajectories
  connectToService(export_trajectory_client_);
  connectToService(import_trajectory_client_);
  ROS_INFO_STREAM("RViz panel " << getName().toStdString() << " services connections have been made");
  Q_EMIT enable(true);
}

void TrajectoryUtils::backButtonHandler()
{
  Q_EMIT configChanged();
  Q_EMIT enable(false);
  params_.request.button_id = 1;
  // Run in a separate thread
  QtConcurrent::run(this, &TrajectoryUtils::sendButton);
}

void TrajectoryUtils::forwardButtonHandler()
{
  Q_EMIT configChanged();
  Q_EMIT enable(false);
  params_.request.button_id = 2;
  // Run in a separate thread
  QtConcurrent::run(this, &TrajectoryUtils::sendButton);
}

void TrajectoryUtils::sendButton()
{
  // Call service
  bool success(trajectory_buffer_client_.call(params_));
  Q_EMIT enable(true);

  if (!success)
  {
    Q_EMIT displayMessageBox("Service call failed", QString::fromStdString(trajectory_buffer_client_.getService()),
                                  "Check the logs!", QMessageBox::Icon::Critical);
  }
  else if (!params_.response.error.empty())
  {
    Q_EMIT displayMessageBox(QString::fromStdString(trajectory_buffer_client_.getService()),
                                  QString::fromStdString(params_.response.error),
                                  "", QMessageBox::Icon::Critical);
  }
}

void TrajectoryUtils::browseFilesToExportTrajectory()
{
  QString file_dir("");
  {
    QFileInfo file(QString::fromStdString(export_filename_.request.file_name));
    if (export_filename_.request.file_name != "" && file.dir().exists())
      file_dir = file.dir().path();
    else
    {
      std::string path = ros::package::getPath("ram_path_planning");
      file_dir = QString::fromStdString(path);
    }
  }

  QFileDialog browser;
  QString file_path = browser.getSaveFileName(this, "Save trajectory", file_dir, "Bag files (*.bag)");

  if (file_path != "")
  {
    if (!file_path.endsWith(".bag"))
      file_path += ".bag";

    Q_EMIT configChanged();
    Q_EMIT enable(false);
    export_filename_.request.file_name = file_path.toStdString();
    // Run in a separate thread
    QtConcurrent::run(this, &TrajectoryUtils::exportTrajectory);
  }
}

void TrajectoryUtils::browseFilesToImportTrajectory()
{
  QString file_dir("");
  {
    QFileInfo file(QString::fromStdString(import_filename_.request.file_name));
    if (import_filename_.request.file_name != "" && file.dir().exists())
      file_dir = file.dir().path();
    else
    {
      std::string path = ros::package::getPath("ram_path_planning");
      file_dir = QString::fromStdString(path);
    }
  }

  QFileDialog browser;
  QString file_path = browser.getOpenFileName(this, "Choose Bag file", file_dir, "Bag files (*.bag)");

  if (file_path != "")
  {
    Q_EMIT configChanged();
    Q_EMIT enable(false);
    import_filename_.request.file_name = file_path.toStdString();
    // Run in a separate thread
    QtConcurrent::run(this, &TrajectoryUtils::importTrajectory);
  }
}

void TrajectoryUtils::exportTrajectory()
{
  // Call service
  bool success(export_trajectory_client_.call(export_filename_));

  Q_EMIT enable(true);
  if (!success)
  {
    Q_EMIT displayMessageBox("Service call failed", QString::fromStdString(export_trajectory_client_.getService()),
                                  "Check the logs!", QMessageBox::Icon::Critical);
  }
  else
  {
    if (export_filename_.response.error.empty())
    {
      Q_EMIT displayMessageBox("The trajectory has been exported to ",
                                      QString::fromStdString(export_filename_.request.file_name), "",
                                      QMessageBox::Icon::Information);
    }
    else
    {
      Q_EMIT displayMessageBox(QString::fromStdString(export_trajectory_client_.getService()),
                                    QString::fromStdString(export_filename_.response.error),
                                    "", QMessageBox::Icon::Critical);
    }
  }
}

void TrajectoryUtils::importTrajectory()
{
  // Call service
  bool success(import_trajectory_client_.call(import_filename_));

  Q_EMIT enable(true);
  if (!success)
  {
    Q_EMIT displayMessageBox("Service call failed", QString::fromStdString(import_trajectory_client_.getService()),
                                  "Check the logs!", QMessageBox::Icon::Critical);
  }
  else
  {
    if (import_filename_.response.error.empty())
    {
      Q_EMIT displayMessageBox("The trajectory has been imported from file ",
                                      QString::fromStdString(import_filename_.request.file_name), "",
                                      QMessageBox::Icon::Information);
    }
    else
    {
      Q_EMIT displayMessageBox(QString::fromStdString(import_trajectory_client_.getService()),
                                    QString::fromStdString(import_filename_.response.error),
                                    "", QMessageBox::Icon::Critical);
    }
  }
}

void TrajectoryUtils::load(const rviz::Config& config)
{
  QString tmp_str("");
  if (config.mapGetString("export_filename", &tmp_str))
    export_filename_.request.file_name = tmp_str.toStdString();
  if (config.mapGetString("import_filename", &tmp_str))
    import_filename_.request.file_name = tmp_str.toStdString();
  rviz::Panel::load(config);
}

void TrajectoryUtils::save(rviz::Config config) const
                           {
  config.mapSetValue("export_filename", QString::fromStdString(export_filename_.request.file_name));
  config.mapSetValue("import_filename", QString::fromStdString(import_filename_.request.file_name));
  rviz::Panel::save(config);
}

void TrajectoryUtils::displayMessageBoxHandler(const QString title,
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
PLUGINLIB_EXPORT_CLASS(ram_qt_guis::TrajectoryUtils, rviz::Panel)
