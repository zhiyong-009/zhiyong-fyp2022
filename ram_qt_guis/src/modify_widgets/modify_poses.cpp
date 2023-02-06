#include <ram_qt_guis/modify_widgets/modify_poses.hpp>

namespace ram_qt_guis
{
ModifyPoses::ModifyPoses(QVBoxLayout* layout,
                         const unsigned mode) :
        layout_(layout),
        mode_(mode)
{
  connect(this, &ModifyPoses::enable, this, &ModifyPoses::setEnabled);
  setObjectName("Modify poses");
  speed_conversion_factor_ = 1;

  if (mode > 1)
  {
    QLabel *label =
        new QLabel(
            "The selection contains poses in Joint mode and Linear mode, it is thus not allowed to modified the robot speed.");
    label->setWordWrap(true);
    layout_->addWidget(label);
  }

  layout_->addWidget(new QLabel("<b>Geometric pose:</b>"));
  QGridLayout* geometric_pose_grid = new QGridLayout;
  layout_->addLayout(geometric_pose_grid);

  // Pose
  pose_widget_ = new QWidget;
  QVBoxLayout *pose_layout = new QVBoxLayout;
  pose_widget_->setLayout(pose_layout);

  pose_modify_ = new QCheckBox;
  pose_ = new Pose("modify", "");
  // Make sure these widgets take as much space as available
  pose_->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));

  pose_abs_rel_ = new QComboBox;
  pose_abs_rel_->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));
  pose_abs_rel_->addItem("Relative");
  pose_abs_rel_->addItem("Absolute");
  connect(pose_abs_rel_, qOverload<int>(&QComboBox::currentIndexChanged), this, &ModifyPoses::poseAbsRel);
  pose_abs_rel_->setCurrentIndex(0);

  pose_layout->addWidget(pose_abs_rel_);
  pose_layout->addWidget(pose_);

  geometric_pose_grid->addWidget(pose_modify_, 0, 0);
  geometric_pose_grid->addWidget(pose_widget_, 0, 1);
  connect(pose_modify_, &QCheckBox::stateChanged, this, &ModifyPoses::modifyPose);
  layout_->addWidget(new QLabel("<b>Pose:</b>"));
  QGridLayout* pose_grid = new QGridLayout;
  layout_->addLayout(pose_grid);

  // Polygon start
  polygon_start_modify_ = new QCheckBox;
  polygon_start_ = new QComboBox;
  polygon_start_->addItem("False");
  polygon_start_->addItem("True");
  // Make sure these widgets take as much space as available
  polygon_start_->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));

  polygon_start_widget_ = new QWidget;
  QVBoxLayout *polygon_start_layout = new QVBoxLayout;
  polygon_start_widget_->setLayout(polygon_start_layout);

  polygon_start_layout->addWidget(new QLabel("Polygon start:"));
  polygon_start_layout->addWidget(polygon_start_);

  pose_grid->addWidget(polygon_start_modify_, 0, 0);
  pose_grid->addWidget(polygon_start_widget_, 0, 1);
  connect(polygon_start_modify_, &QCheckBox::stateChanged, this, &ModifyPoses::modifyPolygonStart);

  // Polygon end
  polygon_end_modify_ = new QCheckBox;
  polygon_end_ = new QComboBox;
  polygon_end_->addItem("False");
  polygon_end_->addItem("True");

  polygon_end_widget_ = new QWidget;
  QVBoxLayout *polygon_end_layout = new QVBoxLayout;
  polygon_end_widget_->setLayout(polygon_end_layout);

  polygon_end_layout->addWidget(new QLabel("Polygon end:"));
  polygon_end_layout->addWidget(polygon_end_);

  pose_grid->addWidget(polygon_end_modify_);
  pose_grid->addWidget(polygon_end_widget_);
  connect(polygon_end_modify_, &QCheckBox::stateChanged, this, &ModifyPoses::modifyPolygonEnd);

  layout_->addWidget(new QLabel("<b>Parameters:</b>"));

  QGridLayout* parameters_grid = new QGridLayout;
  layout_->addLayout(parameters_grid);

  // Movement type
  movement_type_modify_ = new QCheckBox;
  movement_type_ = new QComboBox;
  movement_type_->addItem("Joint");
  movement_type_->addItem("Linear");
  // Make sure these widgets take as much space as available
  movement_type_->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));
  connect(movement_type_modify_, &QCheckBox::stateChanged, this, &ModifyPoses::movementTypeChanged);

  movement_type_widget_ = new QWidget;
  QVBoxLayout *movement_type_layout = new QVBoxLayout;
  movement_type_widget_->setLayout(movement_type_layout);

  movement_type_layout->addWidget(new QLabel("Movement type:"));
  movement_type_layout->addWidget(movement_type_);

  parameters_grid->addWidget(movement_type_modify_, 0, 0);
  parameters_grid->addWidget(movement_type_widget_, 0, 1);
  connect(movement_type_, qOverload<int>(&QComboBox::currentIndexChanged), this, &ModifyPoses::movementTypeChanged);
  connect(movement_type_modify_, &QCheckBox::stateChanged, this, &ModifyPoses::modifyMovementType);

  // Approach type
  approach_type_modify_ = new QCheckBox;
  approach_type_ = new QComboBox;
  approach_type_->addItem("Stop/go");
  approach_type_->addItem("Blend radius");

  approach_type_widget_ = new QWidget;
  QVBoxLayout *approach_type_layout = new QVBoxLayout;
  approach_type_widget_->setLayout(approach_type_layout);

  approach_type_layout->addWidget(new QLabel("Approach type:"));
  approach_type_layout->addWidget(approach_type_);

  parameters_grid->addWidget(approach_type_modify_);
  parameters_grid->addWidget(approach_type_widget_);
  connect(approach_type_modify_, &QCheckBox::stateChanged, this, &ModifyPoses::modifyApproachType);

  // Blend radius
  blend_radius_modify_ = new QCheckBox;
  blend_radius_ = new QSpinBox;
  blend_radius_->setSingleStep(5);
  blend_radius_->setSuffix(" %");

  blend_radius_abs_rel_ = new QComboBox;
  blend_radius_abs_rel_->addItem("Relative");
  blend_radius_abs_rel_->addItem("Absolute");
  connect(blend_radius_abs_rel_, qOverload<int>(&QComboBox::currentIndexChanged), this, &ModifyPoses::blendRadiusAbsRel);
  blend_radius_abs_rel_->setCurrentIndex(1);

  blend_radius_widget_ = new QWidget;
  QVBoxLayout *blend_radius_layout = new QVBoxLayout;
  blend_radius_widget_->setLayout(blend_radius_layout);

  blend_radius_layout->addWidget(new QLabel("Blend radius:"));
  blend_radius_layout->addWidget(blend_radius_abs_rel_);
  blend_radius_layout->addWidget(blend_radius_);

  parameters_grid->addWidget(blend_radius_modify_);
  parameters_grid->addWidget(blend_radius_widget_);
  connect(blend_radius_modify_, &QCheckBox::stateChanged, this, &ModifyPoses::modifyBlendRadius);

  // Speed
  speed_modify_ = new QCheckBox;
  speed_ = new QDoubleSpinBox;
  speed_->setSingleStep(1);
  speed_->setDecimals(3);

  speed_abs_rel_ = new QComboBox;
  speed_abs_rel_->addItem("Relative");
  speed_abs_rel_->addItem("Absolute");
  connect(speed_abs_rel_, qOverload<int>(&QComboBox::currentIndexChanged), this, &ModifyPoses::speedAbsRel);
  speed_abs_rel_->setCurrentIndex(1);

  speed_widget_ = new QWidget;
  QVBoxLayout *speed_layout = new QVBoxLayout;
  speed_widget_->setLayout(speed_layout);

  if (mode < 2)
  {
    speed_layout->addWidget(new QLabel("Speed:"));
    speed_layout->addWidget(speed_abs_rel_);
    speed_layout->addWidget(speed_);

    parameters_grid->addWidget(speed_modify_);
    parameters_grid->addWidget(speed_widget_);
    connect(speed_modify_, &QCheckBox::stateChanged, this, &ModifyPoses::modifySpeed);
  }

  // Laser power
  laser_power_modify_ = new QCheckBox;
  laser_power_ = new QSpinBox;
  laser_power_->setSingleStep(100);
  laser_power_->setSuffix(" W");

  laser_power_abs_rel_ = new QComboBox;
  laser_power_abs_rel_->addItem("Relative");
  laser_power_abs_rel_->addItem("Absolute");
  connect(laser_power_abs_rel_, qOverload<int>(&QComboBox::currentIndexChanged), this, &ModifyPoses::laserPowerAbsRel);
  laser_power_abs_rel_->setCurrentIndex(1);

  laser_power_widget_ = new QWidget;
  QVBoxLayout *laser_power_layout = new QVBoxLayout;
  laser_power_widget_->setLayout(laser_power_layout);

  laser_power_layout->addWidget(new QLabel("Laser power:"));
  laser_power_layout->addWidget(laser_power_abs_rel_);
  laser_power_layout->addWidget(laser_power_);

  parameters_grid->addWidget(laser_power_modify_);
  parameters_grid->addWidget(laser_power_widget_);
  connect(laser_power_modify_, &QCheckBox::stateChanged, this, &ModifyPoses::modifyLaserPower);

  // Feed rate
  feed_rate_modify_ = new QCheckBox;
  feed_rate_ = new QDoubleSpinBox;
  speed_->setDecimals(3);
  feed_rate_->setSingleStep(0.1);
  feed_rate_->setSuffix(" meters/min");

  feed_rate_abs_rel_ = new QComboBox;
  feed_rate_abs_rel_->addItem("Relative");
  feed_rate_abs_rel_->addItem("Absolute");
  connect(feed_rate_abs_rel_, qOverload<int>(&QComboBox::currentIndexChanged), this, &ModifyPoses::feedRateAbsRel);
  feed_rate_abs_rel_->setCurrentIndex(1);

  feed_rate_widget_ = new QWidget;
  QVBoxLayout *feed_rate_layout = new QVBoxLayout;
  feed_rate_widget_->setLayout(feed_rate_layout);

  feed_rate_layout->addWidget(new QLabel("Feed rate:"));
  feed_rate_layout->addWidget(feed_rate_abs_rel_);
  feed_rate_layout->addWidget(feed_rate_);

  parameters_grid->addWidget(feed_rate_modify_);
  parameters_grid->addWidget(feed_rate_widget_);
  connect(feed_rate_modify_, &QCheckBox::stateChanged, this, &ModifyPoses::modifyFeedRate);

  Q_EMIT modifyPose();
  Q_EMIT modifyPolygonStart();
  Q_EMIT modifyPolygonEnd();
  Q_EMIT modifyMovementType();
  Q_EMIT modifyApproachType();
  Q_EMIT modifyBlendRadius();
  Q_EMIT modifySpeed();
  Q_EMIT modifyLaserPower();
  Q_EMIT feed_rate_abs_rel_->currentIndexChanged(0);
  Q_EMIT modifyFeedRate();

  button_box_ = new QDialogButtonBox(QDialogButtonBox::Ok
      | QDialogButtonBox::Cancel);
  layout_->addStretch(1);
  layout_->addWidget(button_box_);

  switch (mode)
  {
    case 0: // All poses are Joint
      movement_type_->setCurrentIndex(0);
      break;
    case 1: // All poses are Linear
      movement_type_->setCurrentIndex(1);
      break;
    default: // Mixed posed
      speed_modify_->setEnabled(false);
      break;
  }
  Q_EMIT movementTypeChanged();
}

ModifyPoses::~ModifyPoses()
{
}

void
ModifyPoses::modifyPose()
{
  pose_widget_->setEnabled(pose_modify_->isChecked());
}

void
ModifyPoses::poseAbsRel(const int)
{
  pose_->setPose(Eigen::Isometry3d::Identity());
}

void
ModifyPoses::modifyPolygonStart()
{
  polygon_start_widget_->setEnabled(polygon_start_modify_->isChecked());
}

void
ModifyPoses::modifyPolygonEnd()
{
  polygon_end_widget_->setEnabled(polygon_end_modify_->isChecked());
}

void
ModifyPoses::modifyMovementType()
{
  movement_type_widget_->setEnabled(movement_type_modify_->isChecked());

  if (movement_type_modify_->isChecked())
    return;

  if (mode_ == 0)
    movement_type_->setCurrentIndex(0);
  else if (mode_ == 1)
    movement_type_->setCurrentIndex(1);

  QStandardItemModel *model = dynamic_cast<QStandardItemModel *>(speed_abs_rel_->model());
  model->item(0, 0)->setEnabled(true);
  model->item(1, 0)->setEnabled(true);
}

void
ModifyPoses::movementTypeChanged()
{
  Q_EMIT enable(false);

  // Change speed
  switch (movement_type_->currentIndex())
  {
    case 0: // Modify poses to: Joint
    {
      speed_->setSuffix(" %");
      speed_conversion_factor_ = 1;

      if (mode_ == 0) // All poses are joint
      {
        QStandardItemModel *model = dynamic_cast<QStandardItemModel *>(speed_abs_rel_->model());
        model->item(0, 0)->setEnabled(true);
        model->item(1, 0)->setEnabled(true);
      }
      else
      {
        QStandardItemModel *model = dynamic_cast<QStandardItemModel *>(speed_abs_rel_->model());
        model->item(0, 0)->setEnabled(false); // Relative
        model->item(1, 0)->setEnabled(true); // Absolute
        speed_abs_rel_->setCurrentIndex(1);
      }
      break;
    }
    case 1: // Modify poses to: Linear
    {
      speed_->setSuffix(" meters/min");
      speed_conversion_factor_ = 1 / 60.0; // meters/min > meters / sec

      if (mode_ == 0) // All poses are joint
      {
        QStandardItemModel *model = dynamic_cast<QStandardItemModel *>(speed_abs_rel_->model());
        model->item(0, 0)->setEnabled(false);
        model->item(1, 0)->setEnabled(true);
        speed_abs_rel_->setCurrentIndex(1);
      }
      else
      {
        QStandardItemModel *model = dynamic_cast<QStandardItemModel *>(speed_abs_rel_->model());
        model->item(0, 0)->setEnabled(true);
        model->item(1, 0)->setEnabled(true);
      }
      break;
    }
  }
  Q_EMIT enable(true);
}

void
ModifyPoses::modifyApproachType()
{
  approach_type_widget_->setEnabled(approach_type_modify_->isChecked());
}

void
ModifyPoses::modifyBlendRadius()
{
  blend_radius_widget_->setEnabled(blend_radius_modify_->isChecked());
}

void
ModifyPoses::blendRadiusAbsRel(const int index)
{
  switch (index)
  {
    case 0:
      blend_radius_->setRange(-100, 100);
      break;
    default:
      blend_radius_->setRange(0, 100);
      break;
  }
}

void
ModifyPoses::modifySpeed()
{
  speed_widget_->setEnabled(speed_modify_->isChecked());
}

void
ModifyPoses::speedAbsRel(const int index)
{
  switch (index)
  {
    case 0:
      speed_->setRange(-100, 100);
      break;
    default:
      speed_->setRange(0.001, 100);
      break;
  }
}

void
ModifyPoses::modifyLaserPower()
{
  laser_power_widget_->setEnabled(laser_power_modify_->isChecked());
}

void
ModifyPoses::laserPowerAbsRel(const int index)
{
  switch (index)
  {
    case 0:
      laser_power_->setRange(-32000, 32000);
      break;
    default:
      laser_power_->setRange(0, 32000);
      break;
  }
}

void
ModifyPoses::modifyFeedRate()
{
  feed_rate_widget_->setEnabled(feed_rate_modify_->isChecked());
}

void
ModifyPoses::feedRateAbsRel(const int index)
{
  switch (index)
  {
    case 0:
      feed_rate_->setRange(-10, 10);
      break;
    default:
      feed_rate_->setRange(0, 10);
      break;
  }
}

}
