#include <ram_qt_guis/display.hpp>

namespace ram_qt_guis
{
Display::Display(QWidget *parent) :
  rviz::Panel(parent)
{
  connect(this, &Display::enable, this, &Display::setEnabled);
  setObjectName("Display");
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
                               new TreeButton("Parameters", tree_widget, category));

    QWidget *widget = new QWidget;
    QVBoxLayout *widget_layout(new QVBoxLayout);
    widget->setLayout(widget_layout);

    display_type_ = new QComboBox;
    display_type_->addItem("Cylinder");
    display_type_->addItem("Wire");
    QHBoxLayout *display_type_layout(new QHBoxLayout);
    display_type_layout->addWidget(new QLabel("Display type"));
    display_type_layout->addWidget(display_type_);
    line_size_ = new QDoubleSpinBox;
    line_size_->setRange(0.01, 100);
    line_size_->setSingleStep(0.2);
    line_size_->setSuffix(" mm");
    QHBoxLayout *line_size_layout(new QHBoxLayout);
    line_size_layout->addWidget(new QLabel("Size"));
    line_size_layout->addWidget(line_size_);

    color_by_ = new QComboBox;
    color_by_->addItem("Layer level");
    color_by_->addItem("Layer index");
    color_by_->addItem("Pose type");
    color_by_->addItem("Movement type");
    color_by_->addItem("Approach type");
    color_by_->addItem("Blend radius");
    color_by_->addItem("Speed");
    color_by_->addItem("Laser power");
    color_by_->addItem("Feed rate");
    QHBoxLayout *color_by_layout(new QHBoxLayout);
    color_by_layout->addWidget(new QLabel("Color by"));
    color_by_layout->addWidget(color_by_);

    widget_layout->addLayout(display_type_layout);
    widget_layout->addLayout(line_size_layout);
    widget_layout->addLayout(color_by_layout);

    QTreeWidgetItem *container = new QTreeWidgetItem();
    container->setDisabled(false);
    category->addChild(container);
    tree_widget->setItemWidget(container, 0, widget);
  }

  {
    QTreeWidgetItem *category(new QTreeWidgetItem);
    tree_widget->addTopLevelItem(category);
    tree_widget->setItemWidget(category, 0,
                               new TreeButton("Axis", tree_widget, category));

    QWidget *widget = new QWidget;
    QVBoxLayout *widget_layout(new QVBoxLayout);
    widget->setLayout(widget_layout);

    display_axis_ = new QCheckBox("Enable");
    display_axis_->setChecked(true);
    axis_size_ = new QDoubleSpinBox;
    axis_size_->setRange(0.01, 100);
    axis_size_->setSingleStep(0.2);
    axis_size_->setSuffix(" mm");
    QWidget *axis_size_widget(new QWidget);
    QHBoxLayout *axis_size_layout(new QHBoxLayout(axis_size_widget));
    axis_size_layout->addWidget(new QLabel("Size"));
    axis_size_layout->addWidget(axis_size_);
    connect(display_axis_, qOverload<int>(&QCheckBox::stateChanged), this, [ = ](int state)
    {
      axis_size_widget->setEnabled(state);
    });

    widget_layout->addWidget(display_axis_);
    widget_layout->addWidget(axis_size_widget);

    QTreeWidgetItem *container = new QTreeWidgetItem();
    container->setDisabled(true);
    category->addChild(container);
    tree_widget->setItemWidget(container, 0, widget);
  }

  {
    QTreeWidgetItem *category(new QTreeWidgetItem);
    tree_widget->addTopLevelItem(category);
    tree_widget->setItemWidget(category, 0,
                               new TreeButton("Labels", tree_widget, category));

    QWidget *widget = new QWidget;
    QVBoxLayout *widget_layout(new QVBoxLayout);
    widget->setLayout(widget_layout);


    display_labels_ = new QCheckBox("Enable");
    display_labels_->setChecked(true);
    label_size_ = new QDoubleSpinBox;
    label_size_->setSingleStep(0.2);
    label_size_->setRange(0.01, 100);
    label_size_->setSuffix(" mm");
    QWidget *labels_size_widget(new QWidget);
    QHBoxLayout *labels_size_layout(new QHBoxLayout(labels_size_widget));
    labels_size_layout->addWidget(new QLabel("Size"));
    labels_size_layout->addWidget(label_size_);
    labels_displayed_ = new QPushButton("Labels displayed");
    connect(display_labels_, qOverload<int>(&QCheckBox::stateChanged), this, [ = ](int state)
    {
      labels_size_widget->setEnabled(state);
      labels_displayed_->setEnabled(state);
    });

    connect(labels_displayed_, &QPushButton::clicked, this, [ = ]()
    {
      QDialog *dialog(new QDialog);
      dialog->setModal(true);
      dialog->setWindowTitle("RViz - Labels displayed");
      QVBoxLayout *dialog_layout(new QVBoxLayout);
      dialog->setLayout(dialog_layout);
      QHBoxLayout *cols_layout(new QHBoxLayout);
      QVBoxLayout *right(new QVBoxLayout);
      QVBoxLayout *left(new QVBoxLayout);
      cols_layout->addLayout(left);
      cols_layout->addLayout(right);

      std::vector<QRadioButton *> radio_buttons;
      radio_buttons.emplace_back(new QRadioButton("Pose number"));
      radio_buttons.emplace_back(new QRadioButton("Pose number within layer"));
      radio_buttons.emplace_back(new QRadioButton("Layer level"));
      radio_buttons.emplace_back(new QRadioButton("Layer index"));

      right->addWidget(new QLabel("Pose label:"));
      for (auto &radio_button : radio_buttons)
        right->addWidget(radio_button);
      right->addStretch(1);

      radio_buttons.at(params_.request.label_type)->setChecked(true);

      QCheckBox *checkbox_pose_type(new QCheckBox("Pose type"));
      QCheckBox *checkbox_laser_power(new QCheckBox("Laser power"));
      QCheckBox *checkbox_feed_rate(new QCheckBox("Feed rate"));
      QCheckBox *checkbox_speed(new QCheckBox("Speed"));
      QCheckBox *checkbox_blend_radius(new QCheckBox("Blend radius"));
      QCheckBox *checkbox_approach_type(new QCheckBox("Approach type"));
      QCheckBox *checkbox_movement_type(new QCheckBox("Movement type"));

      std::vector<QCheckBox *> checkboxes;
      checkboxes.emplace_back(checkbox_pose_type);
      checkboxes.emplace_back(checkbox_laser_power);
      checkboxes.emplace_back(checkbox_feed_rate);
      checkboxes.emplace_back(checkbox_speed);
      checkboxes.emplace_back(checkbox_blend_radius);
      checkboxes.emplace_back(checkbox_approach_type);
      checkboxes.emplace_back(checkbox_movement_type);

      left->addWidget(new QLabel("Extra labels:"));
      for (auto checkbox : checkboxes)
        left->addWidget(checkbox);

      left->addStretch(1);

      QHBoxLayout *all_none_buttons(new QHBoxLayout);
      QPushButton *all(new QPushButton("All"));
      QPushButton *none(new QPushButton("None"));
      all_none_buttons->addWidget(all);
      all_none_buttons->addWidget(none);
      left->addLayout(all_none_buttons);
      connect(all, &QPushButton::clicked, this, [ = ]()
      {
        for (auto &checkbox : checkboxes)
          checkbox->setChecked(true);
      });
      connect(none, &QPushButton::clicked, this, [ = ]()
      {
        for (auto &checkbox : checkboxes)
          checkbox->setChecked(false);
      });

      checkbox_pose_type->setChecked(params_.request.label_pose_type);
      checkbox_laser_power->setChecked(params_.request.label_laser_power);
      checkbox_feed_rate->setChecked(params_.request.label_feed_rate);
      checkbox_speed->setChecked(params_.request.label_speed);
      checkbox_blend_radius->setChecked(params_.request.label_blend_radius);
      checkbox_approach_type->setChecked(params_.request.label_approach_type);
      checkbox_movement_type->setChecked(params_.request.label_movement_type);

      dialog_layout->addLayout(cols_layout);
      QDialogButtonBox *buttons_box(new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel));
      connect(buttons_box, &QDialogButtonBox::accepted, dialog, &QDialog::accept);
      connect(buttons_box, &QDialogButtonBox::rejected, dialog, &QDialog::reject);
      dialog_layout->addWidget(buttons_box);

      if (!dialog->exec())
        return;

      for (std::size_t i(0); i < radio_buttons.size(); ++i)
        if (radio_buttons.at(i)->isChecked())
          params_.request.label_type = i;

      params_.request.label_pose_type = checkbox_pose_type->isChecked();
      params_.request.label_laser_power = checkbox_laser_power->isChecked();
      params_.request.label_feed_rate = checkbox_feed_rate->isChecked();
      params_.request.label_speed = checkbox_speed->isChecked();
      params_.request.label_blend_radius = checkbox_blend_radius->isChecked();
      params_.request.label_approach_type = checkbox_approach_type->isChecked();
      params_.request.label_movement_type = checkbox_movement_type->isChecked();

      Q_EMIT configChanged();
      Q_EMIT display_button_->clicked();
    });

    widget_layout->addWidget(display_labels_);
    widget_layout->addWidget(labels_size_widget);
    widget_layout->addWidget(labels_displayed_);

    QTreeWidgetItem *container = new QTreeWidgetItem();
    container->setDisabled(true);
    category->addChild(container);
    tree_widget->setItemWidget(container, 0, widget);
  }


  {
    QTreeWidgetItem *category(new QTreeWidgetItem);
    tree_widget->addTopLevelItem(category);
    tree_widget->setItemWidget(category, 0,
                               new TreeButton("Transparency", tree_widget, category));

    QWidget *widget = new QWidget;
    QVBoxLayout *widget_layout(new QVBoxLayout);
    widget->setLayout(widget_layout);

    change_alpha_ = new QCheckBox("Enable");
    change_alpha_->setToolTip("Allow to change entry pose, exit pose and any other poses transparency");
    change_alpha_->setChecked(false);
    entry_pose_alpha_ = new QSpinBox;
    exit_pose_alpha_ = new QSpinBox;
    printing_pose_alpha_ = new QSpinBox;
    entry_pose_alpha_->setRange(0, 100);
    exit_pose_alpha_->setRange(0, 100);
    printing_pose_alpha_->setRange(0, 100);
    entry_pose_alpha_->setSingleStep(10);
    exit_pose_alpha_->setSingleStep(10);
    printing_pose_alpha_->setSingleStep(10);
    entry_pose_alpha_->setSuffix(" %");
    exit_pose_alpha_->setSuffix(" %");
    printing_pose_alpha_->setSuffix(" %");

    QLabel *entry_pose_alpha_label = new QLabel("Entry poses");
    QLabel *exit_pose_alpha_label = new QLabel("Exit poses");
    QLabel *printing_poses_alpha_label = new QLabel("Printing poses");
    QWidget *alpha_widget(new QWidget);
    QVBoxLayout *alpha_layout(new QVBoxLayout(alpha_widget));
    QHBoxLayout *entry_layout(new QHBoxLayout);
    entry_layout->addWidget(entry_pose_alpha_label);
    entry_layout->addWidget(entry_pose_alpha_);
    QHBoxLayout *exit_layout(new QHBoxLayout);
    exit_layout->addWidget(exit_pose_alpha_label);
    exit_layout->addWidget(exit_pose_alpha_);
    QHBoxLayout *poses_layout(new QHBoxLayout);
    poses_layout->addWidget(printing_poses_alpha_label);
    poses_layout->addWidget(printing_pose_alpha_);
    alpha_layout->addLayout(entry_layout);
    alpha_layout->addLayout(poses_layout);
    alpha_layout->addLayout(exit_layout);

    alpha_widget->setEnabled(false);
    entry_pose_alpha_->setEnabled(false);
    exit_pose_alpha_->setEnabled(false);
    printing_pose_alpha_->setEnabled(false);

    connect(change_alpha_, qOverload<int>(&QCheckBox::stateChanged), this, [ = ](int state)
    {
      alpha_widget->setEnabled(state);
      entry_pose_alpha_->setEnabled(state);
      exit_pose_alpha_->setEnabled(state);
      printing_pose_alpha_->setEnabled(state);
    });

    widget_layout->addWidget(change_alpha_);
    widget_layout->addWidget(alpha_widget);

    QTreeWidgetItem *container = new QTreeWidgetItem();
    container->setDisabled(false);
    category->addChild(container);
    tree_widget->setItemWidget(container, 0, widget);
  }

  mesh_color_ = new QPushButton("Pick mesh color");

  display_button_ = new QPushButton("Display markers");
  delete_button_ = new QPushButton("Clear markers");
  QHBoxLayout *buttons = new QHBoxLayout;
  buttons->addWidget(display_button_);
  buttons->addWidget(delete_button_);

  layout->addWidget(mesh_color_);
  main_layout->addLayout(buttons);

  connect(mesh_color_, &QPushButton::clicked, this, &Display::pickColor);
  connect(display_button_, &QPushButton::clicked, this, &Display::sendDisplayInformationButtonHandler);
  connect(delete_button_, &QPushButton::clicked, this, &Display::sendDeleteInformationButtonHandler);

  qRegisterMetaType<QMessageBox::Icon>();
  connect(this, &Display::displayMessageBox, this, &Display::displayMessageBoxHandler);

  connect(display_type_, qOverload<int>(&QComboBox::currentIndexChanged), this, &Display::configChanged);
  connect(line_size_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &Display::configChanged);
  connect(color_by_, qOverload<int>(&QComboBox::currentIndexChanged), this, &Display::configChanged);
  connect(display_axis_, &QPushButton::clicked, this, &Display::configChanged);
  connect(axis_size_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &Display::configChanged);
  connect(display_labels_, &QPushButton::clicked, this, &Display::configChanged);
  connect(change_alpha_, &QPushButton::clicked, this, &Display::configChanged);
  connect(label_size_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &Display::configChanged);

  // Setup service clients
  display_client_ = nh_.serviceClient<ram_display::DisplayTrajectory>("ram/display/add_trajectory");
  delete_client_ = nh_.serviceClient<ram_display::DeleteTrajectory>("ram/display/delete_trajectory");
  update_mesh_color_client_ = nh_.serviceClient<ram_display::UpdateMeshColor>("ram/display/update_mesh_color");

  // Check connection of client
  QtConcurrent::run(this, &Display::connectToServices);
}

Display::~Display()
{
}

void Display::connectToService(ros::ServiceClient &client)
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

void Display::connectToServices()
{
  Q_EMIT enable(false);
  connectToService(display_client_);
  connectToService(delete_client_);
  connectToService(update_mesh_color_client_);
  ROS_INFO_STREAM("RViz panel " << getName().toStdString() << " services connections have been made");
  Q_EMIT enable(true);
}

void Display::updateInternalParameters()
{
  params_.request.display_type = display_type_->currentIndex();
  params_.request.line_size = line_size_->value() / 1000.0;
  params_.request.color_mode = color_by_->currentIndex();
  params_.request.display_axis = display_axis_->isChecked();
  params_.request.axis_size = axis_size_->value() / 1000.0;
  params_.request.display_labels = display_labels_->isChecked();
  params_.request.label_size = label_size_->value() / 1000.0;
  if (change_alpha_->isChecked())
  {
    params_.request.entry_pose_alpha = (double) entry_pose_alpha_->value() / 100.0;
    params_.request.exit_pose_alpha = (double) exit_pose_alpha_->value() / 100.0;
    params_.request.additive_pose_alpha = (double) printing_pose_alpha_->value() / 100.0;
  }
  else
  {
    params_.request.entry_pose_alpha = 1.0;
    params_.request.exit_pose_alpha = 1.0;
    params_.request.additive_pose_alpha = 1.0;
  }
  // label_type and labels_* are already filled
  // Color is being modified in pickColor();
}

void Display::pickColor()
{
  Q_EMIT enable(false);

  QColorDialog color;
  color.setModal(true);

  QColor old_color((int)(params_.request.mesh_color.r * 255),
                   (int)(params_.request.mesh_color.g * 255),
                   (int)(params_.request.mesh_color.b * 255),
                   (int)(params_.request.mesh_color.a * 255));
  QColor rgba = color.getColor(old_color, this, "Mesh color", QColorDialog::ShowAlphaChannel);

  if (!rgba.isValid())
  {
    Q_EMIT enable(true);
    return;
  }

  params_.request.mesh_color.r = rgba.red() / 255.0;
  params_.request.mesh_color.g = rgba.green() / 255.0;
  params_.request.mesh_color.b = rgba.blue() / 255.0;
  params_.request.mesh_color.a = rgba.alpha() / 255.0;

  ram_display::UpdateMeshColor srv;
  srv.request.color = params_.request.mesh_color;
  bool status(update_mesh_color_client_.call(srv));
  Q_EMIT enable(true);

  if (!status)
  {
    Q_EMIT displayMessageBox("Update mesh color", "Updating the mesh color failed",
                             QString::fromStdString(update_mesh_color_client_.getService()),
                             QMessageBox::Icon::Critical);
  }
}

void Display::sendDisplayInformationButtonHandler()
{
  Q_EMIT configChanged();
  Q_EMIT enable(false);

  updateInternalParameters();

  // Run in a separate thread
  QtConcurrent::run(this, &Display::sendDisplayInformation);
}

void Display::sendDisplayInformation()
{
  // Call service
  bool success(display_client_.call(params_));
  Q_EMIT enable(true);
  if (!success)
  {
    Q_EMIT displayMessageBox("Service call failed", QString::fromStdString(display_client_.getService()),
                             "Check the logs!", QMessageBox::Icon::Critical);
  }
  else
  {
    if (!params_.response.error.empty())
    {
      Q_EMIT displayMessageBox(QString::fromStdString(display_client_.getService()),
                               QString::fromStdString(params_.response.error),
                               "", QMessageBox::Icon::Critical);
    }
  }
}

void Display::sendDeleteInformationButtonHandler()
{
  Q_EMIT enable(false);
  // Run in a separate thread
  QtConcurrent::run(this, &Display::sendDeleteInformation);
}

void Display::sendDeleteInformation()
{
  // Call service
  ram_display::DeleteTrajectory delete_traj;
  bool success(delete_client_.call(delete_traj));
  Q_EMIT enable(true);

  if (!success)
  {
    Q_EMIT displayMessageBox("Service call failed", QString::fromStdString(delete_client_.getService()),
                             "Check the logs!", QMessageBox::Icon::Critical);
  }
}

void Display::load(const rviz::Config &config)
{
  bool tmp_bool(false);
  int tmp_int(0);
  float tmp_float(0.01);

  if (config.mapGetInt("display_type", &tmp_int))
    display_type_->setCurrentIndex(tmp_int);

  if (config.mapGetFloat("line_size", &tmp_float))
    line_size_->setValue(tmp_float);

  if (config.mapGetInt("color_by", &tmp_int))
    color_by_->setCurrentIndex(tmp_int);

  if (config.mapGetBool("display_axis", &tmp_bool))
    display_axis_->setChecked(tmp_bool);

  if (config.mapGetFloat("axis_size", &tmp_float))
    axis_size_->setValue(tmp_float);

  if (config.mapGetBool("display_labels", &tmp_bool))
    display_labels_->setChecked(tmp_bool);

  if (config.mapGetFloat("labels_size", &tmp_float))
    label_size_->setValue(tmp_float);

  if (config.mapGetBool("change_alpha", &tmp_bool))
    change_alpha_->setChecked(tmp_bool);

  if (config.mapGetInt("label_type", &tmp_int))
    params_.request.label_type = tmp_int;

  if (config.mapGetBool("label_pose_type", &tmp_bool))
    params_.request.label_pose_type = tmp_bool;

  if (config.mapGetBool("label_laser_power", &tmp_bool))
    params_.request.label_laser_power = tmp_bool;

  if (config.mapGetBool("label_feed_rate", &tmp_bool))
    params_.request.label_feed_rate = tmp_bool;

  if (config.mapGetBool("label_speed", &tmp_bool))
    params_.request.label_speed = tmp_bool;

  if (config.mapGetBool("label_blend_radius", &tmp_bool))
    params_.request.label_blend_radius = tmp_bool;

  if (config.mapGetBool("label_approach_type", &tmp_bool))
    params_.request.label_approach_type = tmp_bool;

  if (config.mapGetBool("label_movement_type", &tmp_bool))
    params_.request.label_movement_type = tmp_bool;

  if (config.mapGetFloat("axis_size", &tmp_float))
    axis_size_->setValue(tmp_float);

  if (config.mapGetFloat("mesh_red", &tmp_float))
    params_.request.mesh_color.r = tmp_float;
  else
    params_.request.mesh_color.r = 0.7;

  if (config.mapGetFloat("mesh_green", &tmp_float))
    params_.request.mesh_color.g = tmp_float;
  else
    params_.request.mesh_color.g = 0.7;

  if (config.mapGetFloat("mesh_blue", &tmp_float))
    params_.request.mesh_color.b = tmp_float;
  else
    params_.request.mesh_color.b = 0.7;

  if (config.mapGetFloat("mesh_alpha", &tmp_float))
    params_.request.mesh_color.a = tmp_float;
  else
    params_.request.mesh_color.a = 0.75;

  if (config.mapGetBool("change_alpha", &tmp_bool))
    change_alpha_->setChecked(tmp_bool);
  else
    change_alpha_->setChecked(false);

  if (config.mapGetFloat("entry_pose_alpha", &tmp_float))
    entry_pose_alpha_->setValue(tmp_float);
  else
    entry_pose_alpha_->setValue(100);

  if (config.mapGetFloat("exit_pose_alpha", &tmp_float))
    exit_pose_alpha_->setValue(tmp_float);
  else
    exit_pose_alpha_->setValue(100);

  if (config.mapGetFloat("printing_pose_alpha", &tmp_float))
    printing_pose_alpha_->setValue(tmp_float);
  else
    printing_pose_alpha_->setValue(100);

  rviz::Panel::load(config);
  QtConcurrent::run(this, &Display::sendLoadedInformation);
}

void Display::sendLoadedInformation()
{
  updateInternalParameters();

  // Try to send parameters
  unsigned count(0);
  while (1)
  {
    if (display_client_.exists())
    {
      Q_EMIT enable(false);
      display_client_.call(params_);
      ros::spinOnce();
      Q_EMIT enable(true);
      break;
    }
    ros::Duration(0.5).sleep();

    if (++count > 5)
      break;
  }
}

void Display::save(rviz::Config config) const
{
  config.mapSetValue("display_type", display_type_->currentIndex());
  config.mapSetValue("line_size", line_size_->value());
  config.mapSetValue("color_by", color_by_->currentIndex());
  config.mapSetValue("display_axis", display_axis_->isChecked());
  config.mapSetValue("axis_size", axis_size_->value());
  config.mapSetValue("display_labels", display_labels_->isChecked());
  config.mapSetValue("labels_size", label_size_->value());
  config.mapSetValue("change_alpha", change_alpha_->isChecked());
  config.mapSetValue("entry_pose_alpha", entry_pose_alpha_->value());
  config.mapSetValue("exit_pose_alpha", exit_pose_alpha_->value());
  config.mapSetValue("printing_pose_alpha", printing_pose_alpha_->value());
  config.mapSetValue("label_type", params_.request.label_type);
  config.mapSetValue("label_pose_type", params_.request.label_pose_type);
  config.mapSetValue("label_laser_power", params_.request.label_laser_power);
  config.mapSetValue("label_feed_rate", params_.request.label_feed_rate);
  config.mapSetValue("label_speed", params_.request.label_speed);
  config.mapSetValue("label_blend_radius", params_.request.label_blend_radius);
  config.mapSetValue("label_approach_type", params_.request.label_approach_type);
  config.mapSetValue("label_movement_type", params_.request.label_movement_type);
  config.mapSetValue("mesh_red", params_.request.mesh_color.r);
  config.mapSetValue("mesh_green", params_.request.mesh_color.g);
  config.mapSetValue("mesh_blue", params_.request.mesh_color.b);
  config.mapSetValue("mesh_alpha", params_.request.mesh_color.a);
  rviz::Panel::save(config);
}

void Display::displayMessageBoxHandler(const QString title,
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
PLUGINLIB_EXPORT_CLASS(ram_qt_guis::Display, rviz::Panel)
