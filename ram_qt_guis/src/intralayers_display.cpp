#include <ram_qt_guis/intralayers_display.hpp>

namespace ram_qt_guis
{

IntralayersDisplay::IntralayersDisplay(QWidget *parent) : rviz::Panel(parent)
{
  qRegisterMetaType<QMessageBox::Icon>();
  connect(this, &IntralayersDisplay::displayMessageBox, this, &IntralayersDisplay::displayMessageBoxHandler);

  setObjectName("Layers display");
  setName(objectName());
  QVBoxLayout *layout(new QVBoxLayout(this));
  stacked_widget_ = new QStackedWidget();
  layout->addWidget(stacked_widget_);

  QWidget *buttons_widget_ = new QWidget();
  QHBoxLayout *buttons_layout_ = new QHBoxLayout();
  buttons_widget_->setLayout(buttons_layout_);
  stacked_widget_->addWidget(buttons_widget_);

  range_widget_ = new QWidget;
  stacked_widget_->addWidget(range_widget_);

  first_layer_widget_ = new QWidget;
  first_layer_widget_->setEnabled(false);
  last_layer_widget_ = new QWidget;
  last_layer_widget_->setEnabled(false);

  horizontal_ = new QPushButton("Horizontal");
  connect(horizontal_, &QPushButton::released, this, &IntralayersDisplay::setHorizontalLayout);
  buttons_layout_->addWidget(horizontal_);

  vertical_ = new QPushButton("Vertical");
  connect(vertical_, &QPushButton::released, this, &IntralayersDisplay::setVerticalLayout);
  buttons_layout_->addWidget(vertical_);

  range_slider_ = new ctkRangeSlider();
  range_slider_->setEnabled(false);
  range_slider_->setRange(0, 10);
  connect(range_slider_, &QSlider::sliderReleased, this, &IntralayersDisplay::updateDisplay);
  connect(range_slider_, &ctkRangeSlider::sliderReleased, this, &IntralayersDisplay::displayRangeOfLayers);

  first_layer_spin_box_ = new QSpinBox();
  connect(first_layer_spin_box_, qOverload<int>(&QSpinBox::valueChanged), range_slider_, &ctkRangeSlider::setMinimumValue);

  connect(range_slider_, &ctkRangeSlider::minimumValueChanged, first_layer_spin_box_, &QSpinBox::setValue);
  first_layer_spin_box_->setRange(0, 10);
  // Avoid layers number "inversion"
  connect(first_layer_spin_box_, qOverload<int>(&QSpinBox::valueChanged), this, &IntralayersDisplay::tweakFirstLayer);

  last_layer_spin_box_ = new QSpinBox();
  connect(last_layer_spin_box_, qOverload<int>(&QSpinBox::valueChanged), range_slider_, &ctkRangeSlider::setMaximumValue);
  connect(range_slider_, &ctkRangeSlider::maximumValueChanged, last_layer_spin_box_, &QSpinBox::setValue);
  last_layer_spin_box_->setRange(0, 10);
  last_layer_spin_box_->setValue(last_layer_spin_box_->maximum());
  // Avoid layers number "inversion"
  connect(last_layer_spin_box_, qOverload<int>(&QSpinBox::valueChanged), this, &IntralayersDisplay::tweakLastLayer);

  range_of_layers_check_box_ = new QCheckBox("Enable range selector");
  connect(range_of_layers_check_box_, &QCheckBox::stateChanged, this, &IntralayersDisplay::enableLayersDisplay);
  connect(range_of_layers_check_box_, &QCheckBox::clicked, this, &IntralayersDisplay::configChanged);

  connect(first_layer_spin_box_, qOverload<int>(&QSpinBox::valueChanged), this, &IntralayersDisplay::updateDisplay);
  connect(last_layer_spin_box_, qOverload<int>(&QSpinBox::valueChanged), this, &IntralayersDisplay::updateDisplay);

  // Update the layers view
  connect(this, &IntralayersDisplay::layerRangeChanged, this, &IntralayersDisplay::changeLayerRange);

  //Allows to get the amount of layers in the trajectory
  subcriber_ = nh_.subscribe("ram/information/trajectory", 1, &IntralayersDisplay::callBackTrajectoryInformation,
                             this);
  display_client_ = nh_.serviceClient<ram_display::DisplayRangeOfLayers>("ram/display/display_range_of_layers");

  updateInternalParameters();
  QtConcurrent::run(this, &IntralayersDisplay::connectToServicesAndPublishers);
}

IntralayersDisplay::~IntralayersDisplay()
{
}

void IntralayersDisplay::changeLayout()
{
  horizontal_->setMinimumSize(1, 1);
  vertical_->setMinimumSize(1, 1);
  stacked_widget_->setCurrentIndex(1);
  Q_EMIT configChanged();
}

void IntralayersDisplay::setHorizontalLayout()
{
  QHBoxLayout *layout = new QHBoxLayout;
  range_widget_->setLayout(layout);
  range_slider_->setOrientation(Qt::Orientation::Horizontal);
  range_of_layers_check_box_->setText("");
  QHBoxLayout *first_layer_layout = new QHBoxLayout;
  QHBoxLayout *last_layer_layout = new QHBoxLayout;
  first_layer_widget_->setLayout(first_layer_layout);
  last_layer_widget_->setLayout(last_layer_layout);
  first_layer_layout->addWidget(first_layer_spin_box_);
  last_layer_layout->addWidget(last_layer_spin_box_);
  layout->addWidget(range_of_layers_check_box_);
  layout->addWidget(first_layer_widget_);
  layout->addWidget(range_slider_);
  layout->addWidget(last_layer_widget_);
  changeLayout();

  layout_ = "horizontal";
}

void IntralayersDisplay::setVerticalLayout()
{
  QHBoxLayout *layout = new QHBoxLayout;
  range_widget_->setLayout(layout);
  range_slider_->setOrientation(Qt::Orientation::Vertical);
  layout->addWidget(range_slider_);
  QVBoxLayout *right_layout_ = new QVBoxLayout();
  layout->addLayout(right_layout_);
  QHBoxLayout *first_layer_layout = new QHBoxLayout;
  QHBoxLayout *last_layer_layout = new QHBoxLayout;
  first_layer_widget_->setLayout(first_layer_layout);
  last_layer_widget_->setLayout(last_layer_layout);
  first_layer_layout->addWidget(new QLabel("First Layer"));
  first_layer_layout->addWidget(first_layer_spin_box_);
  last_layer_layout->addWidget(new QLabel("Last Layer"));
  last_layer_layout->addWidget(last_layer_spin_box_);
  right_layout_->addWidget(last_layer_widget_);
  right_layout_->addStretch(1);
  right_layout_->addWidget(range_of_layers_check_box_);
  right_layout_->addStretch(1);
  right_layout_->addWidget(first_layer_widget_);
  changeLayout();

  layout_ = "vertical";
}

void IntralayersDisplay::displayRangeOfLayers()
{
  params_.request.first_layer = range_slider_->minimumValue();
  params_.request.last_layer = range_slider_->maximumValue();

  QtConcurrent::run(this, &IntralayersDisplay::sendDisplayInformation);
}

void IntralayersDisplay::tweakFirstLayer()
{
  if (first_layer_spin_box_->value() > last_layer_spin_box_->value())
    last_layer_spin_box_->setValue(first_layer_spin_box_->value());
}

void IntralayersDisplay::tweakLastLayer()
{
  if (first_layer_spin_box_->value() > last_layer_spin_box_->value())
    first_layer_spin_box_->setValue(last_layer_spin_box_->value());
}

void IntralayersDisplay::updateDisplay()
{
  Q_EMIT configChanged();
  Q_EMIT enable(false);

  updateInternalParameters();

  QtConcurrent::run(this, &IntralayersDisplay::sendDisplayInformation);
}

void IntralayersDisplay::changeLayerRange(int number_of_layers)
{
  first_layer_spin_box_->setRange(0, number_of_layers - 1);
  last_layer_spin_box_->setRange(0, first_layer_spin_box_->maximum());
  range_slider_->setMinimum(0);
  range_slider_->setMaximum(first_layer_spin_box_->maximum());
}

void IntralayersDisplay::updateInternalParameters()
{
  params_.request.display_range_of_layers = range_of_layers_check_box_->isChecked();
  params_.request.first_layer = first_layer_spin_box_->value();
  params_.request.last_layer = last_layer_spin_box_->value();
}

void IntralayersDisplay::load(const rviz::Config &config)
{
  QString tmp_str;

  if (config.mapGetString("layout", &tmp_str))
  {
    if (tmp_str == "vertical")
      Q_EMIT vertical_->released();

    else if (tmp_str == "horizontal")
      Q_EMIT horizontal_->released();
  }
  rviz::Panel::load(config);
}

void IntralayersDisplay::save(rviz::Config config) const
{
  config.mapSetValue("layout", layout_);
  rviz::Panel::save(config);
}

void IntralayersDisplay::sendDisplayInformation()
{
  if (!display_client_.waitForExistence(ros::Duration(0.1)))
  {
    Q_EMIT displayMessageBox("Cannot call service", QString::fromStdString(display_client_.getService()) + " does not exist",
                             "Check the logs!", QMessageBox::Icon::Critical);
    return;
  }

  bool success(display_client_.call(params_));
  Q_EMIT enable(true);

  if (!success)
  {
    Q_EMIT displayMessageBox("Service call failed", QString::fromStdString(display_client_.getService()),
                             "Check the logs!", QMessageBox::Icon::Critical);
  }
  else
  {
    if (params_.response.error == "Trajectory is too small. Two poses are required")
    {
      // Ignore this specific message to allow setting the numbers of
      // layers displayed without having a trajectory
    }
    else if (!params_.response.error.empty())
    {
      Q_EMIT displayMessageBox(QString::fromStdString(display_client_.getService()),
                               QString::fromStdString(params_.response.error),
                               "", QMessageBox::Icon::Critical);
    }
  }
}

void
IntralayersDisplay::callBackTrajectoryInformation(const ram_msgs::AdditiveManufacturingTrajectoryInfoConstPtr &msg)
{
  Q_EMIT layerRangeChanged(int(msg->number_of_layers_levels));
}

void IntralayersDisplay::enableLayersDisplay(bool state)
{
  first_layer_widget_->setEnabled(state);
  last_layer_widget_->setEnabled(state);
  range_slider_->setEnabled(state);

  params_.request.display_range_of_layers = state;

  QtConcurrent::run(this, &IntralayersDisplay::sendDisplayInformation);
}

void IntralayersDisplay::displayMessageBoxHandler(const QString title,
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

void IntralayersDisplay::connectToService(ros::ServiceClient &client)
{
  while (nh_.ok())
  {
    if (client.waitForExistence(ros::Duration(2)))
    {
      ROS_INFO_STREAM(
      "RViz panel " << getName().toStdString() << " connected to the service "
      << client.getService());
      break;
    }
    else
    {
      ROS_ERROR_STREAM(
      "RViz panel " << getName().toStdString() << " could not connect to ROS service: "
      << client.getService());
      ros::Duration(1).sleep();
    }
  }
}

void IntralayersDisplay::checkForPublishers()
{
  while (nh_.ok())
  {
    ros::Duration(1).sleep();

    if (subcriber_.getNumPublishers() != 0)
    {
      ROS_INFO_STREAM(
      "RViz panel " << getName().toStdString() << " topic " << subcriber_.getTopic()
      << " has at least one publisher");
      break;
    }
    else
    {
      ROS_ERROR_STREAM(
      "RViz panel " << getName().toStdString() << " topic " << subcriber_.getTopic()
      << " has zero publishers!");
    }
  }
}

void IntralayersDisplay::connectToServicesAndPublishers()
{
  Q_EMIT enable(false);
  connectToService(display_client_);
  ROS_INFO_STREAM("RViz panel " << getName().toStdString() << " services connections have been made");
  checkForPublishers();
  Q_EMIT enable(true);
}

}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(ram_qt_guis::IntralayersDisplay, rviz::Panel)
