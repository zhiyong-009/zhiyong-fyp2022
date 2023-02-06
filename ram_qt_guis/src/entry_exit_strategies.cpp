#include <ram_qt_guis/entry_exit_strategies.hpp>

namespace ram_qt_guis
{
EntryExitStrategies::EntryExitStrategies(QWidget *parent) :
  rviz::Panel(parent)
{
  connect(this, &EntryExitStrategies::enable, this, &EntryExitStrategies::setEnabled);
  setObjectName("Entry, exit strategies");
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
                               new TreeButton("Entry poses", tree_widget, category));

    QWidget *widget = new QWidget;
    QGridLayout *widget_layout(new QGridLayout);
    widget->setLayout(widget_layout);

    entry_number_of_poses_ = new QSpinBox;
    entry_number_of_poses_->setRange(1, 100);

    entry_angle_ = new QDoubleSpinBox;
    entry_angle_->setRange(-180, 180);
    entry_angle_->setSingleStep(10);
    entry_angle_->setSuffix(" °");

    entry_distance_ = new QDoubleSpinBox;
    entry_distance_->setRange(0, 1000);
    entry_distance_->setSingleStep(1);
    entry_distance_->setSuffix(" mm");

    entry_movement_type_ = new QComboBox;
    entry_movement_type_->addItem("Joint");
    entry_movement_type_->addItem("Linear");

    entry_approach_type_ = new QComboBox;
    entry_approach_type_->addItem("Stop/go");
    entry_approach_type_->addItem("Blend radius");

    entry_blend_radius_ = new QSpinBox;
    entry_blend_radius_->setRange(0, 100);
    entry_blend_radius_->setSingleStep(5);
    entry_blend_radius_->setSuffix(" %");

    entry_move_speed_ = new QDoubleSpinBox;
    entry_move_speed_ ->setRange(0.001, 100);
    entry_move_speed_ ->setSingleStep(0.1);
    entry_move_speed_ ->setDecimals(3);

    widget_layout->addWidget(new QLabel("Number of poses:"), 0, 0);
    widget_layout->addWidget(entry_number_of_poses_, 0, 1);
    widget_layout->addWidget(new QLabel("Angle:"));
    widget_layout->addWidget(entry_angle_);
    widget_layout->addWidget(new QLabel("Distance:"));
    widget_layout->addWidget(entry_distance_);
    widget_layout->addWidget(new QLabel("Movement type:"));
    widget_layout->addWidget(entry_movement_type_);
    widget_layout->addWidget(new QLabel("Approach type:"));
    widget_layout->addWidget(entry_approach_type_);
    widget_layout->addWidget(new QLabel("Blend radius:"));
    widget_layout->addWidget(entry_blend_radius_);
    widget_layout->addWidget(new QLabel("Speed:"));
    widget_layout->addWidget(entry_move_speed_);

    QTreeWidgetItem *container = new QTreeWidgetItem();
    container->setDisabled(true);
    category->addChild(container);
    tree_widget->setItemWidget(container, 0, widget);

    connect(entry_movement_type_, qOverload<int>(&QComboBox::currentIndexChanged), this, [ = ]()
    {
      Q_EMIT enable(false);
      // Change speed
      switch (entry_movement_type_->currentIndex())
      {
        case 0: // Joint
          {
            entry_move_speed_->setSuffix("");
            entry_conversion_factor_ = 1;
            break;
          }
        case 1: // Linear
          {
            entry_move_speed_->setSuffix(" meters/min");
            entry_conversion_factor_ = 1 / 60.0; // meters/min > meters / sec
            break;
          }
        default:
          {
            ROS_ERROR_STREAM("movementTypeChanged unknown movement type");
            break;
          }
      }
      Q_EMIT enable(true);
    });

    connect(entry_approach_type_, qOverload<int>(&QComboBox::currentIndexChanged), this, [ = ]()
    {
      Q_EMIT enable(false);
      switch (entry_approach_type_->currentIndex())
      {
        case 0: // Stop-go
          entry_blend_radius_->setEnabled(false);
          break;
        case 1: // Blend radius
          entry_blend_radius_->setEnabled(true);
          break;
      }
      Q_EMIT enable(true);
    });
  }

  {
    QTreeWidgetItem *category(new QTreeWidgetItem);
    tree_widget->addTopLevelItem(category);
    tree_widget->setItemWidget(category, 0,
                               new TreeButton("Exit poses", tree_widget, category));

    QWidget *widget = new QWidget;
    QGridLayout *widget_layout(new QGridLayout);
    widget->setLayout(widget_layout);

    exit_number_of_poses_ = new QSpinBox;
    exit_number_of_poses_->setRange(1, 100);

    exit_angle_ = new QDoubleSpinBox;
    exit_angle_->setRange(-180, 180);
    exit_angle_->setSingleStep(10);
    exit_angle_->setSuffix(" °");

    exit_distance_ = new QDoubleSpinBox;
    exit_distance_->setRange(0, 1000);
    exit_distance_->setSingleStep(1);
    exit_distance_->setSuffix(" mm");

    exit_movement_type_ = new QComboBox;
    exit_movement_type_->addItem("Joint");
    exit_movement_type_->addItem("Linear");

    exit_approach_type_ = new QComboBox;
    exit_approach_type_->addItem("Stop/go");
    exit_approach_type_->addItem("Blend radius");

    exit_blend_radius_ = new QSpinBox;
    exit_blend_radius_->setRange(0, 100);
    exit_blend_radius_->setSingleStep(5);
    exit_blend_radius_->setSuffix(" %");

    exit_move_speed_ = new QDoubleSpinBox;
    exit_move_speed_ ->setRange(0.001, 100);
    exit_move_speed_ ->setSingleStep(0.1);
    exit_move_speed_ ->setDecimals(3);

    widget_layout->addWidget(new QLabel("Number of poses:"), 0, 0);
    widget_layout->addWidget(exit_number_of_poses_, 0, 1);
    widget_layout->addWidget(new QLabel("Angle:"));
    widget_layout->addWidget(exit_angle_);
    widget_layout->addWidget(new QLabel("Distance:"));
    widget_layout->addWidget(exit_distance_);
    widget_layout->addWidget(new QLabel("Movement type:"));
    widget_layout->addWidget(exit_movement_type_);
    widget_layout->addWidget(new QLabel("Approach type:"));
    widget_layout->addWidget(exit_approach_type_);
    widget_layout->addWidget(new QLabel("Blend radius:"));
    widget_layout->addWidget(exit_blend_radius_);
    widget_layout->addWidget(new QLabel("Speed:"));
    widget_layout->addWidget(exit_move_speed_);

    QTreeWidgetItem *container = new QTreeWidgetItem();
    container->setDisabled(true);
    category->addChild(container);
    tree_widget->setItemWidget(container, 0, widget);

    connect(exit_movement_type_, qOverload<int>(&QComboBox::currentIndexChanged), this, [ = ]()
    {
      Q_EMIT enable(false);
      // Change speed
      switch (exit_movement_type_->currentIndex())
      {
        case 0: // Joint
          {
            exit_move_speed_->setSuffix("");
            exit_conversion_factor_ = 1;
            break;
          }
        case 1: // Linear
          {
            exit_move_speed_->setSuffix(" meters/min");
            exit_conversion_factor_ = 1 / 60.0; // meters/min > meters / sec
            break;
          }
        default:
          {
            ROS_ERROR_STREAM("movementTypeChanged unknown movement type");
            break;
          }
      }
      Q_EMIT enable(true);
    });

    connect(exit_approach_type_, qOverload<int>(&QComboBox::currentIndexChanged), this, [ = ]()
    {
      Q_EMIT enable(false);
      switch (exit_approach_type_->currentIndex())
      {
        case 0: // Stop-go
          exit_blend_radius_->setEnabled(false);
          break;
        case 1: // Blend radius
          exit_blend_radius_->setEnabled(true);
          break;
      }
      Q_EMIT enable(true);
    });
  }

  entry_exit_button_ = new QPushButton("Entry / exit strategies");
  layout->addWidget(entry_exit_button_);
  qRegisterMetaType<QMessageBox::Icon>();
  connect(this, &EntryExitStrategies::displayMessageBox, this, &EntryExitStrategies::displayMessageBoxHandler);

  connect(entry_exit_button_, &QPushButton::clicked, this, &EntryExitStrategies::sendEntryExitParameters);

  connect(entry_number_of_poses_, qOverload<int>(&QSpinBox::valueChanged), this, &EntryExitStrategies::configChanged);
  connect(entry_angle_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &EntryExitStrategies::configChanged);
  connect(entry_distance_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &EntryExitStrategies::configChanged);
  connect(entry_movement_type_, qOverload<int>(&QComboBox::currentIndexChanged), this, &EntryExitStrategies::configChanged);
  connect(entry_approach_type_, qOverload<int>(&QComboBox::currentIndexChanged), this, &EntryExitStrategies::configChanged);
  connect(entry_blend_radius_, qOverload<int>(&QSpinBox::valueChanged), this, &EntryExitStrategies::configChanged);
  connect(entry_move_speed_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &EntryExitStrategies::configChanged);

  connect(exit_number_of_poses_, qOverload<int>(&QSpinBox::valueChanged), this, &EntryExitStrategies::configChanged);
  connect(exit_angle_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &EntryExitStrategies::configChanged);
  connect(exit_distance_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &EntryExitStrategies::configChanged);
  connect(entry_movement_type_, qOverload<int>(&QComboBox::currentIndexChanged), this, &EntryExitStrategies::configChanged);
  connect(entry_approach_type_, qOverload<int>(&QComboBox::currentIndexChanged), this, &EntryExitStrategies::configChanged);
  connect(entry_blend_radius_, qOverload<int>(&QSpinBox::valueChanged), this, &EntryExitStrategies::configChanged);
  connect(entry_move_speed_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &EntryExitStrategies::configChanged);

  // Setup service clients
  entry_parameters_client_ = nh_.serviceClient<ram_utils::EntryExitParameters>("ram/information/entry_parameters");
  exit_parameters_client_ = nh_.serviceClient<ram_utils::EntryExitParameters>("ram/information/exit_parameters");

  // Check connection of clients
  connectToServices();
}

EntryExitStrategies::~EntryExitStrategies()
{
}

void EntryExitStrategies::updateInternalParameters()
{
  Q_EMIT configChanged();
  entry_parameters_.request.angle = entry_angle_->value() * M_PI / 180.0; // Convert in radians
  entry_parameters_.request.distance = entry_distance_->value() / 1000.0; // Convert meters in millimeters
  entry_parameters_.request.number_of_poses = entry_number_of_poses_->value();
  entry_parameters_.request.params.movement_type = entry_movement_type_->currentIndex();
  entry_parameters_.request.params.approach_type = entry_approach_type_->currentIndex();
  entry_parameters_.request.params.blend_radius = entry_blend_radius_->value();
  entry_parameters_.request.params.printing_move_speed = entry_move_speed_->value() * entry_conversion_factor_;
  entry_parameters_.request.params.non_printing_move_speed = entry_parameters_.request.params.printing_move_speed;

  exit_parameters_.request.angle = exit_angle_->value() * M_PI / 180.0;
  exit_parameters_.request.distance = exit_distance_->value() / 1000.0;
  exit_parameters_.request.number_of_poses = exit_number_of_poses_->value();
  exit_parameters_.request.params.movement_type = exit_movement_type_->currentIndex();
  exit_parameters_.request.params.approach_type = exit_approach_type_->currentIndex();
  exit_parameters_.request.params.blend_radius = exit_blend_radius_->value();
  exit_parameters_.request.params.printing_move_speed = exit_move_speed_->value() * exit_conversion_factor_;
  exit_parameters_.request.params.non_printing_move_speed = exit_parameters_.request.params.printing_move_speed;
}

void EntryExitStrategies::connectToService(ros::ServiceClient &client)
{
  Q_EMIT enable(false);

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

void EntryExitStrategies::connectToServices()
{
  Q_EMIT enable(false);

  connectToService(entry_parameters_client_);
  connectToService(exit_parameters_client_);

  ROS_INFO_STREAM("RViz panel " << getName().toStdString() << " services connections have been made");
  Q_EMIT enable(true);
}

void EntryExitStrategies::displayMessageBoxHandler(const QString title,
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

void EntryExitStrategies::sendEntryExitParameters()
{
  Q_EMIT enable(false);

  // Call service
  updateInternalParameters();
  bool success(entry_parameters_client_.call(entry_parameters_));

  if (!success)
  {
    Q_EMIT displayMessageBox("Service call failed", QString::fromStdString(entry_parameters_client_.getService()),
                             "Check the logs!", QMessageBox::Icon::Critical);
    Q_EMIT enable(true);
    return;
  }

  // Call service
  updateInternalParameters();
  success = exit_parameters_client_.call(exit_parameters_);

  if (!success)
  {
    Q_EMIT displayMessageBox("Service call failed",
                             QString::fromStdString(exit_parameters_client_.getService()),
                             "Check the logs!", QMessageBox::Icon::Critical);
    Q_EMIT enable(true);
    return;
  }

  Q_EMIT enable(true);
}

void EntryExitStrategies::load(const rviz::Config &config)
{
  int tmp_int(0);
  float tmp_float(0.01);
  Q_EMIT configChanged();

  if (config.mapGetInt("entry_number_of_poses", &tmp_int))
    entry_number_of_poses_->setValue(tmp_int);
  else
    entry_number_of_poses_->setValue(1);

  if (config.mapGetFloat("entry_angle", &tmp_float))
    entry_angle_->setValue(tmp_float);
  else
    entry_angle_->setValue(90);

  if (config.mapGetFloat("entry_distance", &tmp_float))
    entry_distance_->setValue(tmp_float);
  else
    entry_distance_->setValue(10);

  if (config.mapGetInt("entry_movement_type", &tmp_int))
    entry_movement_type_->setCurrentIndex(tmp_int);
  else
    entry_movement_type_->setCurrentIndex(1);

  if (config.mapGetInt("entry_approach_type", &tmp_int))
    entry_approach_type_->setCurrentIndex(tmp_int);
  else
    entry_approach_type_->setCurrentIndex(1);

  if (config.mapGetInt("entry_blend_radius", &tmp_int))
    entry_blend_radius_->setValue(tmp_int);
  else
    entry_blend_radius_->setValue(100);

if (config.mapGetFloat("entry_move_speed", &tmp_float))
    entry_move_speed_->setValue(tmp_float);
  else
    entry_move_speed_->setValue(1.0);

  if (config.mapGetInt("exit_number_of_poses", &tmp_int))
    exit_number_of_poses_->setValue(tmp_int);
  else
    exit_number_of_poses_->setValue(1);

  if (config.mapGetFloat("exit_angle", &tmp_float))
    exit_angle_->setValue(tmp_float);
  else
    exit_angle_->setValue(90);

  if (config.mapGetFloat("exit_distance", &tmp_float))
    exit_distance_->setValue(tmp_float);
  else
    exit_distance_->setValue(10);

  if (config.mapGetInt("exit_movement_type", &tmp_int))
    exit_movement_type_->setCurrentIndex(tmp_int);
  else
    exit_movement_type_->setCurrentIndex(1);

  if (config.mapGetInt("exit_approach_type", &tmp_int))
    exit_approach_type_->setCurrentIndex(tmp_int);
  else
    exit_approach_type_->setCurrentIndex(1);

  if (config.mapGetInt("exit_blend_radius", &tmp_int))
    exit_blend_radius_->setValue(tmp_int);
  else
    exit_blend_radius_->setValue(100);

if (config.mapGetFloat("exit_move_speed", &tmp_float))
    exit_move_speed_->setValue(tmp_float);
  else
    exit_move_speed_->setValue(1.0);

  Q_EMIT sendEntryExitParameters();
  rviz::Panel::load(config);
}

void EntryExitStrategies::save(rviz::Config config) const
{
  config.mapSetValue("entry_number_of_poses", entry_number_of_poses_->value());
  config.mapSetValue("entry_angle", entry_angle_->value());
  config.mapSetValue("entry_distance", entry_distance_->value());
  config.mapSetValue("entry_movement_type", entry_movement_type_->currentIndex());
  config.mapSetValue("entry_approach_type", entry_approach_type_->currentIndex());
  config.mapSetValue("entry_blend_radius", entry_blend_radius_->value());
  config.mapSetValue("entry_move_speed", entry_move_speed_->value());

  config.mapSetValue("exit_number_of_poses", exit_number_of_poses_->value());
  config.mapSetValue("exit_angle", exit_angle_->value());
  config.mapSetValue("exit_distance", exit_distance_->value());
  config.mapSetValue("exit_movement_type", exit_movement_type_->currentIndex());
  config.mapSetValue("exit_approach_type", exit_approach_type_->currentIndex());
  config.mapSetValue("exit_blend_radius", exit_blend_radius_->value());
  config.mapSetValue("exit_move_speed", exit_move_speed_->value());
  rviz::Panel::save(config);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ram_qt_guis::EntryExitStrategies, rviz::Panel)
