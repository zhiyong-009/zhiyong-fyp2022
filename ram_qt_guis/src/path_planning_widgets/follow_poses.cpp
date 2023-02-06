#include <ram_qt_guis/path_planning_widgets/follow_poses.hpp>

namespace ram_qt_guis
{
FollowPosesWidget::FollowPosesWidget()

{
  setObjectName("FollowPoses");

  main_layout_ = new QVBoxLayout(this);

  QHBoxLayout *file = new QHBoxLayout;
  QPushButton *file_explorer = new QPushButton;
  file_explorer->setText("...");
  file_explorer->setMaximumSize(30, 30);
  file_ = new QLineEdit;
  file->addWidget(file_);
  file->addWidget(file_explorer);
  connect(file_explorer, &QPushButton::released, this, &FollowPosesWidget::browseFiles);

  duplicate_layer_ = new QCheckBox;
  duplicate_layer_->setText("Duplicate layers");
  duplicate_layer_->setChecked(false);

  number_of_layers_ = new QSpinBox;
  number_of_layers_->setRange(2, 10000);
  number_of_layers_->setToolTip("The number of layers in the final trajectory");
  QHBoxLayout *number_of_layers_layout = new QHBoxLayout;
  number_of_layers_layout->addWidget(new QLabel("Number of layers:"));
  number_of_layers_layout->addWidget(number_of_layers_);

  height_between_layers_ = new QDoubleSpinBox;
  height_between_layers_->setRange(0.001, 1000);
  height_between_layers_->setSuffix(" mm");
  height_between_layers_->setSingleStep(0.1);
  height_between_layers_->setDecimals(3);
  QHBoxLayout *height_between_layers_layout = new QHBoxLayout;
  height_between_layers_layout->addWidget(new QLabel("Height between layers:"));
  height_between_layers_layout->addWidget(height_between_layers_);

  invert_one_of_two_layers_ = new QCheckBox("Invert one of two layers");

  duplicate_layer_widget_ = new QWidget;
  QVBoxLayout *duplicate_layer_layout = new QVBoxLayout;
  duplicate_layer_widget_->setLayout(duplicate_layer_layout);
  duplicate_layer_layout->addLayout(number_of_layers_layout);
  duplicate_layer_layout->addLayout(height_between_layers_layout);
  duplicate_layer_layout->addWidget(invert_one_of_two_layers_);

  arc_points_ = new QSpinBox;
  arc_points_->setRange(4, 1000);
  QHBoxLayout *arc_points_layout = new QHBoxLayout;
  arc_points_layout->addWidget(new QLabel("Arc points (SVG file only):"));
  arc_points_layout->addWidget(arc_points_);

  rotate_poses_ = new QSpinBox;
  rotate_poses_->setRange(0, 10000);
  QHBoxLayout *rotate_poses_layout = new QHBoxLayout;
  rotate_poses_layout->addWidget(new QLabel("Rotate poses (SVG file only):"));
  rotate_poses_layout->addWidget(rotate_poses_);

  // Main layout
  main_layout_->addWidget(new QLabel("SVG or YAML file:"));
  main_layout_->addLayout(file);
  main_layout_->addWidget(duplicate_layer_);
  main_layout_->addWidget(duplicate_layer_widget_);
  main_layout_->addLayout(arc_points_layout);
  main_layout_->addLayout(rotate_poses_layout);
  main_layout_->addStretch(1);

  connect(file_, &QLineEdit::textChanged, this, &FollowPosesWidget::valueChanged);
  connect(duplicate_layer_, &QPushButton::clicked, this, &FollowPosesWidget::valueChanged);
  connect(duplicate_layer_, &QPushButton::clicked, this, &FollowPosesWidget::enableDisableDuplicateLayers);
  connect(number_of_layers_, qOverload<int>(&QSpinBox::valueChanged), this, &FollowPosesWidget::valueChanged);
  connect(invert_one_of_two_layers_, &QCheckBox::stateChanged, this, &FollowPosesWidget::valueChanged);
  connect(height_between_layers_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &FollowPosesWidget::valueChanged);
  connect(arc_points_, qOverload<int>(&QSpinBox::valueChanged), this, &FollowPosesWidget::valueChanged);
  connect(rotate_poses_, qOverload<int>(&QSpinBox::valueChanged), this, &FollowPosesWidget::valueChanged);
}

FollowPosesWidget::~FollowPosesWidget()
{
}

void FollowPosesWidget::browseFiles()
{
  QString file_dir("");
  {
    QFileInfo file(file_->text());
    if (!file_->text().isEmpty() && file.dir().exists())
      file_dir = file.dir().path();
    else
    {
      std::string path = ros::package::getPath("ram_path_planning");
      file_dir = QString::fromStdString(path);
    }
  }

  QFileDialog browser;
  browser.setOption(QFileDialog::DontUseNativeDialog, true);
  QString file_path = browser.getOpenFileName(this, "Choose SVG or YAML file", file_dir,
                                              "SVG and YAML files (*.svg *.SVG *.yaml *.YAML *.yml *.YML)");
  if (file_path != "")
    file_->setText(file_path);
}

void FollowPosesWidget::enableDisableDuplicateLayers()
{
  duplicate_layer_widget_->setEnabled(duplicate_layer_->isChecked());
}

void FollowPosesWidget::load(const rviz::Config& config)
{
  QString tmp_str("");
  bool tmp_bool(false);
  int tmp_int(0);
  float tmp_float(0.01);

  if (config.mapGetString(objectName() + "_file", &tmp_str))
    file_->setText(tmp_str);

  if (config.mapGetBool(objectName() + "_duplicate_layer", &tmp_bool))
    duplicate_layer_->setChecked(tmp_bool);
  else
    duplicate_layer_->setChecked(false);

  if (config.mapGetInt(objectName() + "_number_of_layers", &tmp_int))
    number_of_layers_->setValue(tmp_int);

  if (config.mapGetFloat(objectName() + "_height_between_layers", &tmp_float))
    height_between_layers_->setValue(tmp_float);
  else
    height_between_layers_->setValue(default_height_between_layers_);

  if (config.mapGetBool(objectName() + "_invert_one_of_two_layers", &tmp_bool))
    invert_one_of_two_layers_->setChecked(tmp_bool);
  else
    invert_one_of_two_layers_->setChecked(false);

  if (config.mapGetInt(objectName() + "_arc_points", &tmp_int))
    arc_points_->setValue(tmp_int);
  else
    arc_points_->setValue(10);

  if (config.mapGetInt(objectName() + "_rotate_poses", &tmp_int))
    rotate_poses_->setValue(tmp_int);
  else
    rotate_poses_->setValue(0);

  Q_EMIT enableDisableDuplicateLayers();
}

void FollowPosesWidget::save(rviz::Config config) const
                             {
  config.mapSetValue(objectName() + "_file", file_->text());
  config.mapSetValue(objectName() + "_duplicate_layer", duplicate_layer_->isChecked());
  config.mapSetValue(objectName() + "_number_of_layers", number_of_layers_->value());
  config.mapSetValue(objectName() + "_height_between_layers", height_between_layers_->value());
  config.mapSetValue(objectName() + "_invert_one_of_two_layers", invert_one_of_two_layers_->isChecked());
  config.mapSetValue(objectName() + "_arc_points", arc_points_->value());
  config.mapSetValue(objectName() + "_rotate_poses", rotate_poses_->value());
}

std::string FollowPosesWidget::fillGoal(ram_path_planning::FollowPosesGoal &goal)
{
  if (file_->text().isEmpty())
    return "File name is not specified.";

  goal.file = file_->text().toStdString();
  goal.duplicate_layer = duplicate_layer_->isChecked();
  goal.number_of_layers = number_of_layers_->value();
  goal.height_between_layers = height_between_layers_->value() / 1000.0;
  goal.invert_one_of_two_layers = invert_one_of_two_layers_->isChecked();
  goal.arc_points = arc_points_->value();
  goal.rotate_poses = rotate_poses_->value();
  return "";
}

}
