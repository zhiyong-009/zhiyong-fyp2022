#include <ram_qt_guis/path_planning_widgets/contours.hpp>

namespace ram_qt_guis
{
ContoursWidget::ContoursWidget()

{
  setObjectName("ContoursAlgorithm");

  main_layout_ = new QVBoxLayout(this);

  QHBoxLayout *file = new QHBoxLayout;
  QPushButton *file_explorer = new QPushButton;
  file_explorer->setText("...");
  file_explorer->setMaximumSize(30, 30);
  file_ = new QLineEdit;
  connect(file_, &QLineEdit::textChanged, this, &ContoursWidget::fileChanged);
  file->addWidget(file_);
  file->addWidget(file_explorer);
  connect(file_explorer, &QPushButton::released, this, &ContoursWidget::browseFiles);

  number_of_layers_ = new QSpinBox;
  number_of_layers_->setRange(1, 10000);
  QHBoxLayout *number_of_layers_layout = new QHBoxLayout;
  number_of_layers_layout->addWidget(new QLabel("Number of layers:"));
  number_of_layers_layout->addWidget(number_of_layers_);
  number_of_layers_widget_ = new QWidget;
  number_of_layers_widget_->setLayout(number_of_layers_layout);

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

  slicing_direction_x_ = new QDoubleSpinBox;
  slicing_direction_x_->setRange(-20000, 20000);
  slicing_direction_x_->setSingleStep(0.01);
  slicing_direction_x_->setDecimals(6);
  slicing_direction_x_->setMaximumWidth(80);
  slicing_direction_x_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  slicing_direction_y_ = new QDoubleSpinBox;
  slicing_direction_y_->setMinimum(slicing_direction_x_->minimum());
  slicing_direction_y_->setMaximum(slicing_direction_x_->maximum());
  slicing_direction_y_->setSingleStep(slicing_direction_x_->singleStep());
  slicing_direction_y_->setDecimals(slicing_direction_x_->decimals());
  slicing_direction_y_->setMaximumWidth(slicing_direction_x_->maximumWidth());
  slicing_direction_y_->setSizePolicy(slicing_direction_x_->sizePolicy());

  slicing_direction_z_ = new QDoubleSpinBox;
  slicing_direction_z_->setMinimum(slicing_direction_x_->minimum());
  slicing_direction_z_->setMaximum(slicing_direction_x_->maximum());
  slicing_direction_z_->setSingleStep(slicing_direction_x_->singleStep());
  slicing_direction_z_->setDecimals(slicing_direction_x_->decimals());
  slicing_direction_z_->setMaximumWidth(slicing_direction_x_->maximumWidth());
  slicing_direction_z_->setSizePolicy(slicing_direction_x_->sizePolicy());

  QHBoxLayout* vector_layout = new QHBoxLayout;
  vector_layout->addWidget(new QLabel("X"));
  vector_layout->addWidget(slicing_direction_x_);
  vector_layout->addStretch(1);
  vector_layout->addWidget(new QLabel("Y"));
  vector_layout->addWidget(slicing_direction_y_);
  vector_layout->addStretch(1);
  vector_layout->addWidget(new QLabel("Z"));
  vector_layout->addWidget(slicing_direction_z_);
  QVBoxLayout *slicing_direction_layout = new QVBoxLayout;
  slicing_direction_layout->addWidget(new QLabel("Slicing direction:"));
  slicing_direction_layout->addLayout(vector_layout);
  slicing_direction_widget_ = new QWidget;
  slicing_direction_widget_->setLayout(slicing_direction_layout);

  // Main layout
  main_layout_->addWidget(new QLabel("YAML or mesh file:"));
  main_layout_->addLayout(file);
  main_layout_->addWidget(number_of_layers_widget_);
  main_layout_->addLayout(height_between_layers_layout);
  main_layout_->addLayout(deposited_material_width_layout);
  main_layout_->addWidget(slicing_direction_widget_);
  main_layout_->addStretch(1);

  connect(file_, &QLineEdit::textChanged, this, &ContoursWidget::valueChanged);
  connect(number_of_layers_, qOverload<int>(&QSpinBox::valueChanged), this, &ContoursWidget::valueChanged);
  connect(height_between_layers_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &ContoursWidget::valueChanged);
  connect(deposited_material_width_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &ContoursWidget::valueChanged);
  connect(slicing_direction_x_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &ContoursWidget::valueChanged);
  connect(slicing_direction_y_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &ContoursWidget::valueChanged);
  connect(slicing_direction_z_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &ContoursWidget::valueChanged);
}

ContoursWidget::~ContoursWidget()
{
}

void ContoursWidget::browseFiles()
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
      this, tr("Choose YAML or mesh file"), file_dir,
      tr("YAML and mesh files (*.yaml *.YAML *.yml *.YML *.ply *.PLY *.stl *.STL *.obj *.OBJ)"));
  if (file_path != "")
    file_->setText(file_path);
}

void ContoursWidget::fileChanged()
{
  std::string file_extension(ram_utils::fileExtension(file_->text().toStdString()));
  if (!strcasecmp(file_extension.c_str(), "yaml") || !strcasecmp(file_extension.c_str(), "yml"))
  // YAML
  {
    number_of_layers_widget_->setEnabled(true);
    slicing_direction_widget_->setEnabled(false);
  }
  else if (!strcasecmp(file_extension.c_str(), "ply") || !strcasecmp(file_extension.c_str(), "stl")
      || !strcasecmp(file_extension.c_str(), "obj"))
  // Mesh
  {
    number_of_layers_widget_->setEnabled(false);
    slicing_direction_widget_->setEnabled(true);
  }
  else
  // Enable everything
  {
    number_of_layers_widget_->setEnabled(true);
    slicing_direction_widget_->setEnabled(true);
  }
}

void ContoursWidget::load(const rviz::Config& config)
{
  QString tmp_str("");
  int tmp_int(0);
  float tmp_float(0.01);

  if (config.mapGetString(objectName() + "file", &tmp_str))
    file_->setText(tmp_str);

  if (config.mapGetInt(objectName() + "number_of_layers", &tmp_int))
    number_of_layers_->setValue(tmp_int);

  if (config.mapGetFloat(objectName() + "height_between_layers", &tmp_float))
    height_between_layers_->setValue(tmp_float);
  else
    height_between_layers_->setValue(default_height_between_layers_);

  if (config.mapGetFloat(objectName() + "deposited_material_width", &tmp_float))
    deposited_material_width_->setValue(tmp_float);
  else
    deposited_material_width_->setValue(default_deposited_material_width_);

  if (config.mapGetFloat(objectName() + "slicing_direction_x", &tmp_float))
    slicing_direction_x_->setValue(tmp_float);
  else
    slicing_direction_x_->setValue(default_slicing_direction_x_);

  if (config.mapGetFloat(objectName() + "slicing_direction_y", &tmp_float))
    slicing_direction_y_->setValue(tmp_float);
  else
    slicing_direction_y_->setValue(default_slicing_direction_y_);

  if (config.mapGetFloat(objectName() + "slicing_direction_z", &tmp_float))
    slicing_direction_z_->setValue(tmp_float);
  else
    slicing_direction_z_->setValue(default_slicing_direction_z_);
}

void ContoursWidget::save(rviz::Config config) const
                          {
  config.mapSetValue(objectName() + "file", file_->text());
  config.mapSetValue(objectName() + "number_of_layers", number_of_layers_->value());
  config.mapSetValue(objectName() + "height_between_layers", height_between_layers_->value());
  config.mapSetValue(objectName() + "deposited_material_width", deposited_material_width_->value());
  config.mapSetValue(objectName() + "slicing_direction_x", slicing_direction_x_->value());
  config.mapSetValue(objectName() + "slicing_direction_y", slicing_direction_y_->value());
  config.mapSetValue(objectName() + "slicing_direction_z", slicing_direction_z_->value());
}

std::string ContoursWidget::fillGoal(ram_path_planning::ContoursGoal &goal)
{
  if (file_->text().isEmpty())
    return "File name is not specified.";

  goal.file = file_->text().toStdString();
  goal.number_of_layers = number_of_layers_->value();
  goal.height_between_layers = height_between_layers_->value() / 1000.0;
  goal.deposited_material_width = deposited_material_width_->value() / 1000.0;

  //Slicing direction
  goal.slicing_direction.x = slicing_direction_x_->value();
  goal.slicing_direction.y = slicing_direction_y_->value();
  goal.slicing_direction.z = slicing_direction_z_->value();
  return "";
}

}
