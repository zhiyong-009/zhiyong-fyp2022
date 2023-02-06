#include <ram_qt_guis/path_planning_widgets/profile.hpp>

namespace ram_qt_guis
{
ProfileWidget::ProfileWidget()

{
  setObjectName("ProfileAlgorithm");

  main_layout_ = new QVBoxLayout(this);

  QHBoxLayout *file = new QHBoxLayout;
  QPushButton *file_explorer = new QPushButton;
  file_explorer->setText("...");
  file_explorer->setMaximumSize(30, 30);
  file_ = new QLineEdit;
  file->addWidget(file_);
  file->addWidget(file_explorer);
  connect(file_explorer, &QPushButton::released, this, &ProfileWidget::browseFiles);

  height_between_layers_ = new QDoubleSpinBox;
  height_between_layers_->setRange(0.001, 1000);
  height_between_layers_->setSuffix(" mm");
  height_between_layers_->setSingleStep(0.1);
  height_between_layers_->setDecimals(3);
  QHBoxLayout *height_between_layers_layout = new QHBoxLayout;
  height_between_layers_layout->addWidget(new QLabel("Height between layers:"));
  height_between_layers_layout->addWidget(height_between_layers_);

  deposited_material_width_ = new QDoubleSpinBox;
  deposited_material_width_->setRange(0.001, 1000);
  deposited_material_width_->setSuffix(" mm");
  deposited_material_width_->setSingleStep(0.1);
  deposited_material_width_->setDecimals(3);
  QHBoxLayout *deposited_material_width_layout = new QHBoxLayout;
  deposited_material_width_layout->addWidget(new QLabel("Deposited material width:"));
  deposited_material_width_layout->addWidget(deposited_material_width_);

  slice_along_path_ = new QCheckBox("Slice along path");
  towards_interior_ = new QCheckBox("Towards interior");

  number_of_passes_ = new QSpinBox;
  number_of_passes_->setRange(1, 250);
  QHBoxLayout *number_of_passes_layout_ = new QHBoxLayout;
  number_of_passes_layout_->addWidget(new QLabel("Number of passes:"));
  number_of_passes_layout_->addWidget(number_of_passes_);

  angle_percentage_ = new QSpinBox;
  angle_percentage_->setRange(0, 100);
  angle_percentage_->setSingleStep(10);
  angle_percentage_->setSuffix(" %");
  QHBoxLayout *angle_percentage_layout_ = new QHBoxLayout;
  angle_percentage_layout_->addWidget(new QLabel("Angle percentage:"));
  angle_percentage_layout_->addWidget(angle_percentage_);

  angle_type_ = new QComboBox;
  angle_type_->addItem("Constant");
  angle_type_->addItem("Opposite");
  angle_type_->addItem("Zero opposite");
  QHBoxLayout *angle_type_layout = new QHBoxLayout;
  angle_type_layout->addWidget(new QLabel("Angle type:"));
  angle_type_layout->addWidget(angle_type_);

  arc_points_ = new QSpinBox;
  arc_points_->setRange(4, 1000);
  QHBoxLayout *arc_points_layout = new QHBoxLayout;
  arc_points_layout->addWidget(new QLabel("Arc points:"));
  arc_points_layout->addWidget(arc_points_);

  // Main layout
  main_layout_->addWidget(new QLabel("SVG file:"));
  main_layout_->addLayout(file);
  main_layout_->addLayout(height_between_layers_layout);
  main_layout_->addLayout(deposited_material_width_layout);
  main_layout_->addWidget(slice_along_path_);
  main_layout_->addWidget(towards_interior_);
  main_layout_->addLayout(number_of_passes_layout_);
  main_layout_->addLayout(angle_percentage_layout_);
  main_layout_->addLayout(angle_type_layout);
  main_layout_->addLayout(arc_points_layout);
  main_layout_->addStretch(1);

  connect(file_, &QLineEdit::textChanged, this, &ProfileWidget::valueChanged);
  connect(height_between_layers_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &ProfileWidget::valueChanged);
  connect(deposited_material_width_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &ProfileWidget::valueChanged);
  connect(slice_along_path_, &QCheckBox::stateChanged, this, &ProfileWidget::valueChanged);
  connect(towards_interior_, &QCheckBox::stateChanged, this, &ProfileWidget::valueChanged);
  connect(number_of_passes_, qOverload<int>(&QSpinBox::valueChanged), this, &ProfileWidget::valueChanged);
  connect(angle_percentage_, qOverload<int>(&QSpinBox::valueChanged), this, &ProfileWidget::valueChanged);
  connect(angle_type_, qOverload<int>(&QComboBox::currentIndexChanged), this, &ProfileWidget::valueChanged);
  connect(arc_points_, qOverload<int>(&QSpinBox::valueChanged), this, &ProfileWidget::valueChanged);
}

ProfileWidget::~ProfileWidget()
{
}

void ProfileWidget::browseFiles()
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
  QString file_path = browser.getOpenFileName(
      this, tr("Choose SVG file"), file_dir,
      tr("SVG files (*.svg *.SVG)"));
  if (file_path != "")
    file_->setText(file_path);
}

void ProfileWidget::load(const rviz::Config& config)
{
  QString tmp_str("");
  int tmp_int(0);
  float tmp_float(0.01);
  bool tmp_bool(false);

  if (config.mapGetString(objectName() + "_file", &tmp_str))
    file_->setText(tmp_str);

  if (config.mapGetFloat(objectName() + "_height_between_layers", &tmp_float))
    height_between_layers_->setValue(tmp_float);
  else
    height_between_layers_->setValue(1);

  if (config.mapGetFloat(objectName() + "_deposited_material_width", &tmp_float))
    deposited_material_width_->setValue(tmp_float);
  else
    deposited_material_width_->setValue(1);

  if (config.mapGetBool(objectName() + "_slice_along_path", &tmp_bool))
    slice_along_path_->setChecked(tmp_bool);

  if (config.mapGetBool(objectName() + "_towards_interior", &tmp_bool))
    towards_interior_->setChecked(tmp_bool);

  if (config.mapGetInt(objectName() + "_number_of_passes", &tmp_int))
    number_of_passes_->setValue(tmp_int);
  else
    number_of_passes_->setValue(1);

  if (config.mapGetInt(objectName() + "_angle_percentage", &tmp_int))
    angle_percentage_->setValue(tmp_int);
  else
    angle_percentage_->setValue(100);

  if (config.mapGetInt(objectName() + "_angle_type", &tmp_int))
    angle_type_->setCurrentIndex(tmp_int);
  else
    angle_type_->setCurrentIndex(0);

  if (config.mapGetInt(objectName() + "_arc_points", &tmp_int))
    arc_points_->setValue(tmp_int);
  else
    arc_points_->setValue(50);
}

void ProfileWidget::save(rviz::Config config) const
                          {
  config.mapSetValue(objectName() + "_file", file_->text());
  config.mapSetValue(objectName() + "_height_between_layers", height_between_layers_->value());
  config.mapSetValue(objectName() + "_deposited_material_width", deposited_material_width_->value());
  config.mapSetValue(objectName() + "_slice_along_path", slice_along_path_->isChecked());
  config.mapSetValue(objectName() + "_towards_interior", towards_interior_->isChecked());
  config.mapSetValue(objectName() + "_number_of_passes", number_of_passes_->value());
  config.mapSetValue(objectName() + "_angle_percentage", angle_percentage_->value());
  config.mapSetValue(objectName() + "_angle_type", angle_type_->currentIndex());
  config.mapSetValue(objectName() + "_arc_points", arc_points_->value());
}

std::string ProfileWidget::fillGoal(ram_path_planning::ProfileGoal &goal)
{
  if (file_->text().isEmpty())
    return "File name is not specified.";

  goal.deposited_material_width = deposited_material_width_->value() / 1000.0;
  goal.file = file_->text().toStdString();
  goal.height_between_layers = height_between_layers_->value() / 1000.0;
  goal.number_of_passes = number_of_passes_->value();
  goal.towards_interior = towards_interior_->isChecked();
  goal.slice_along_path = slice_along_path_->isChecked();
  goal.angle_percentage = (double) angle_percentage_->value() / 100.0;
  goal.angle_type = angle_type_->currentIndex();
  goal.arc_points = arc_points_->value();
  return "";
}

}
