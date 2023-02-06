#include <ram_qt_guis/path_planning_widgets/polygon_offsets.hpp>

namespace ram_qt_guis
{
PolygonOffsetsWidget::PolygonOffsetsWidget()
{
  setObjectName("PolygonOffsetsAlgorithm");

  main_layout_ = new QVBoxLayout(this);

  QHBoxLayout *file = new QHBoxLayout;
  QPushButton *file_explorer = new QPushButton;
  file_explorer->setToolTip("Select the input file (either a SVG or a YAML(YML) file");
  file_explorer->setText("...");
  file_explorer->setMaximumSize(30, 30);
  file_ = new QLineEdit;
  file->addWidget(file_);
  file->addWidget(file_explorer);
  connect(file_explorer, &QPushButton::released, this, &PolygonOffsetsWidget::browseFiles);

  number_of_layers_ = new QSpinBox;
  number_of_layers_->setToolTip("Select the layer quantity needed");
  number_of_layers_->setRange(1, 50000);
  QHBoxLayout *number_of_layers_layout = new QHBoxLayout;
  number_of_layers_layout->addWidget(new QLabel("Number of layers:"));
  number_of_layers_layout->addWidget(number_of_layers_);

  height_between_layers_ = new QDoubleSpinBox;
  height_between_layers_->setToolTip("Select the height needed");
  height_between_layers_->setRange(0.001, 1000);
  height_between_layers_->setSuffix(" mm");
  height_between_layers_->setSingleStep(0.1);
  height_between_layers_->setDecimals(3);
  QHBoxLayout *height_between_layers_layout = new QHBoxLayout;
  height_between_layers_layout->addWidget(new QLabel("Height between layers:"));
  height_between_layers_layout->addWidget(height_between_layers_);

  deposited_material_width_ = new QDoubleSpinBox;
  deposited_material_width_->setToolTip("Select the space bewteen each pass needed");
  deposited_material_width_->setRange(0.001, 1000);
  deposited_material_width_->setSuffix(" mm");
  deposited_material_width_->setSingleStep(0.1);
  deposited_material_width_->setDecimals(3);
  QHBoxLayout *deposited_material_width_layout = new QHBoxLayout;
  deposited_material_width_layout->addWidget(new QLabel("Deposited material width:"));
  deposited_material_width_layout->addWidget(deposited_material_width_);

  safe_distance_ = new QDoubleSpinBox;
  safe_distance_->setRange(0, 1000);
  safe_distance_->setSingleStep(0.1);
  safe_distance_->setDecimals(2);
  safe_distance_->setSuffix(" mm");
  safe_distance_->setToolTip(
      "Set the distance which will not be heated by the laser during the process");
  QHBoxLayout *safe_distance_layout = new QHBoxLayout;
  safe_distance_layout->addWidget(new QLabel("Connection safe distance:"));
  safe_distance_layout->addWidget(safe_distance_);

  QWidget *connection_type_widget = new QWidget;
  QVBoxLayout *connection_layout = new QVBoxLayout;
  connection_type_widget->setEnabled(false);
  connection_type_widget->setLayout(connection_layout);

  connection_type_ = new QComboBox;
  connection_type_->addItem("Angle");
  connection_type_->addItem("Distance");
  connection_type_->setToolTip("Set the distance between the current pass and the pass + 1, by default set to 0 mm\n"
                                "or set the angle between the current pass and the pass + 1, by default set to Pi/2");
  QHBoxLayout *connection_type_layout = new QHBoxLayout;
  connection_type_layout->addWidget(new QLabel("Connection type: "));
  connection_type_layout->addWidget(connection_type_);

  connection_value_ = new QDoubleSpinBox;
  connection_value_->setSingleStep(1);
  connection_value_->setToolTip(connection_type_->toolTip());

  QHBoxLayout *connection_value_layout = new QHBoxLayout;
  connection_value_layout->addWidget(new QLabel("Connection value: "));
  connection_value_layout->addWidget(connection_value_);

  connection_layout->addLayout(safe_distance_layout);
  connection_layout->addLayout(connection_type_layout);
  connection_layout->addLayout(connection_value_layout);

  connect(connection_type_, qOverload<int>(&QComboBox::currentIndexChanged), connection_value_, [=]
  {
    if (connection_type_->currentIndex() == 0)
    {
      connection_value_->setSuffix(" °");
      connection_value_->setValue(90);
      connection_value_->setRange(-180, 180);
      connection_value_->setDecimals(1);
      connection_value_->setSingleStep(1);
    }
    else
    {
      connection_value_->setSuffix(" mm");
      connection_value_->setValue(0);
      connection_value_->setRange(-100, 100);
      connection_value_->setDecimals(2);
      connection_value_->setSingleStep(0.2);
    }
  });

  change_angle_ = new QCheckBox ("Change the connection (only for closed paths)");
  change_angle_->setToolTip("Allow to change the connection between passes");
  connect(change_angle_, &QCheckBox::stateChanged, connection_type_widget, [=]
  {
    connection_type_widget->setEnabled(change_angle_->isChecked());
  });

  offset_factor_ = new QDoubleSpinBox;
  offset_factor_->setRange(0, 1000);
  offset_factor_->setSingleStep(0.1);
  offset_factor_->setDecimals(2);
  offset_factor_->setValue(0);
  offset_factor_->setSuffix(" mm");
  offset_factor_->setToolTip("Allow to offset location between current pass and pass + 1");
  QHBoxLayout *offset_factor_layout = new QHBoxLayout;
  offset_factor_layout->addWidget(new QLabel("Offset factor:"));
  offset_factor_layout->addWidget(offset_factor_);
  connection_layout->addLayout(offset_factor_layout);

  towards_interior_ = new QCheckBox("Towards interior");
  avoid_trajectories_crossing_ = new QCheckBox("Avoid trajectories crossing (only open paths)");
  avoid_trajectories_crossing_->setToolTip("if enable, this option allows to avoid crossing with origin trajectory and offset trajectory\n"
                                           "To do that, an exit pose is inserted at the end of the origin trajectory and an entry pose is\n"
                                           "inserted at the beginning of the offset trajectory");
  discontinous_trajectory_ = new QCheckBox("Discontinous trajectory");
  discontinous_trajectory_->setToolTip(
      "Define whether the trajectory is continous or not (continous = the process does not stop)");
  reverse_origin_path_ = new QCheckBox("Manually reverse origin path");
  reverse_origin_path_->setToolTip("Manually reverse the origin trajectory");
  automatic_reverse_path_ = new QCheckBox("Automatically reverse origin path");
  automatic_reverse_path_->setToolTip("Define if the first pass (origin trajectory) is automatically reversed (some cases need it)");

  connect(automatic_reverse_path_, &QCheckBox::stateChanged, reverse_origin_path_, [=]
  {
    reverse_origin_path_->setEnabled(!automatic_reverse_path_->isChecked());
  });

  connect(reverse_origin_path_, &QCheckBox::stateChanged, automatic_reverse_path_, [=]
  {
    automatic_reverse_path_->setEnabled(!reverse_origin_path_->isChecked());
  });

  number_of_passes_ = new QSpinBox;
  number_of_passes_->setRange(1, 300);
  number_of_passes_->setToolTip("Define the number of passes, in fact the shape width in terms of offset number");
  QHBoxLayout *number_of_passes_layout = new QHBoxLayout;
  number_of_passes_layout->addWidget(new QLabel("Number of passes:"));
  number_of_passes_layout->addWidget(number_of_passes_);

  join_type_ = new QComboBox;
  join_type_->setToolTip("Define the join topology of the offset shape : \n"
                         "Square: Approximated joint by squaring is applied at all convex edge joins\n"
                         "Round: Approximated edge by a series of arc chord s\n"
                         "Miter: Approximated Joint by a peak");
  join_type_->addItem("Square");
  join_type_->addItem("Round");
  join_type_->addItem("Miter");
  QHBoxLayout *join_type_layout = new QHBoxLayout;
  join_type_layout->addWidget(new QLabel("Join type:"));
  join_type_layout->addWidget(join_type_);

  end_type_ = new QComboBox;
  end_type_->setToolTip("Define the end of each segment : \n"
                        "Closed polygon: Ends are joined using the JoinType value and the path filled as a polygon\n"
                        "Closed line: Ends are joined using the JoinType value and the path filled as a polyline\n"
                        "Open square: Ends are squared off and extended delta units\n"
                        "Open round: Ends are rounded off and extended delta units\n"
                        "Open butt: Ends are squared off with no extension.");
  end_type_->addItem("Closed polygon");
  end_type_->addItem("Closed line");
  end_type_->addItem("Open butt");
  end_type_->addItem("Open square");
  end_type_->addItem("Open round");
  QHBoxLayout *end_type_layout = new QHBoxLayout;
  end_type_layout->addWidget(new QLabel("End type:"));
  end_type_layout->addWidget(end_type_);

  arc_tolerance_ = new QDoubleSpinBox;
  arc_tolerance_->setToolTip(
      "Define the arc tolerance, specify an acceptable imprecision during the offsetting operation.\n"
      "Smaller value increases 'smoothness'");
  arc_tolerance_->setRange(0.25, 1000);
  arc_tolerance_->setValue(750);
  arc_tolerance_->setDecimals(0);
  QHBoxLayout *arc_tolerance_layout = new QHBoxLayout;
  arc_tolerance_layout->addWidget(new QLabel("Arc tolerance:"));
  arc_tolerance_layout->addWidget(arc_tolerance_);

  miter_limit_ = new QDoubleSpinBox;
  miter_limit_->setToolTip(
      "Set the maximum distance in multiples that vertices can be offset from their original positions, "
      "is applyed on square and miter joins");
  miter_limit_->setRange(0.1, 1000);
  miter_limit_->setValue(2);
  miter_limit_->setDecimals(1);
  QHBoxLayout *miter_limit_layout = new QHBoxLayout;
  miter_limit_layout->addWidget(new QLabel("Miter limit:"));
  miter_limit_layout->addWidget(miter_limit_);

  arc_points_ = new QSpinBox;
  arc_points_->setToolTip("Define the SVG import curves accuracy");
  arc_points_->setRange(4, 1000);
  QHBoxLayout *arc_points_layout = new QHBoxLayout;
  arc_points_layout->addWidget(new QLabel("Arc points:"));
  arc_points_layout->addWidget(arc_points_);

  // Main layout
  main_layout_->addWidget(new QLabel("Input file (SVG, YAML or YML):"));
  main_layout_->addLayout(file);
  main_layout_->addLayout(number_of_layers_layout);
  main_layout_->addLayout(height_between_layers_layout);
  main_layout_->addLayout(deposited_material_width_layout);
  main_layout_->addLayout(arc_points_layout);
  main_layout_->addWidget(discontinous_trajectory_);
  main_layout_->addWidget(avoid_trajectories_crossing_);
  main_layout_->addLayout(number_of_passes_layout);
  main_layout_->addWidget(towards_interior_);
  main_layout_->addLayout(end_type_layout);
  main_layout_->addLayout(join_type_layout);
  main_layout_->addLayout(arc_tolerance_layout);
  main_layout_->addLayout(miter_limit_layout);
  main_layout_->addLayout(safe_distance_layout);
  main_layout_->addWidget(automatic_reverse_path_);
  main_layout_->addWidget(reverse_origin_path_);
  main_layout_->addWidget(change_angle_);
  main_layout_->addWidget(connection_type_widget);
  main_layout_->addStretch(1);

  connect(join_type_, qOverload<int>(&QComboBox::currentIndexChanged), this, &PolygonOffsetsWidget::joinTypeChanged);
  joinTypeChanged();

  connect(file_, &QLineEdit::textChanged, this, &PolygonOffsetsWidget::valueChanged);
  connect(number_of_layers_, qOverload<int>(&QSpinBox::valueChanged), this, &PolygonOffsetsWidget::valueChanged);
  connect(height_between_layers_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          &PolygonOffsetsWidget::valueChanged);
  connect(deposited_material_width_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          &PolygonOffsetsWidget::valueChanged);
  connect(connection_type_, qOverload<int>(&QComboBox::currentIndexChanged), this, &PolygonOffsetsWidget::valueChanged);
  connect(change_angle_, qOverload<int>(&QCheckBox::stateChanged), this, &PolygonOffsetsWidget::valueChanged);
  connect(connection_value_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          &PolygonOffsetsWidget::valueChanged);
  connect(safe_distance_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &PolygonOffsetsWidget::valueChanged);
  connect(offset_factor_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &PolygonOffsetsWidget::valueChanged);
  connect(towards_interior_, qOverload<int>(&QCheckBox::stateChanged), this, &PolygonOffsetsWidget::valueChanged);
  connect(reverse_origin_path_, qOverload<int>(&QCheckBox::stateChanged), this, &PolygonOffsetsWidget::valueChanged);
  connect(discontinous_trajectory_, qOverload<int>(&QCheckBox::stateChanged), this,
          &PolygonOffsetsWidget::valueChanged);
  connect(avoid_trajectories_crossing_, qOverload<int>(&QCheckBox::stateChanged), this,
          &PolygonOffsetsWidget::valueChanged);
  connect(automatic_reverse_path_, qOverload<int>(&QCheckBox::stateChanged), this, &PolygonOffsetsWidget::valueChanged);
  connect(number_of_passes_, qOverload<int>(&QSpinBox::valueChanged), this, &PolygonOffsetsWidget::valueChanged);
  connect(join_type_, qOverload<int>(&QComboBox::currentIndexChanged), this, &PolygonOffsetsWidget::valueChanged);
  connect(end_type_, qOverload<int>(&QComboBox::currentIndexChanged), this, &PolygonOffsetsWidget::valueChanged);
  connect(arc_tolerance_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &PolygonOffsetsWidget::valueChanged);
  connect(miter_limit_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &PolygonOffsetsWidget::valueChanged);
  connect(arc_points_, qOverload<int>(&QSpinBox::valueChanged), this, &PolygonOffsetsWidget::valueChanged);
}

PolygonOffsetsWidget::~PolygonOffsetsWidget()
{
}

void PolygonOffsetsWidget::browseFiles()
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

void PolygonOffsetsWidget::load(const rviz::Config &config)
{
  QString tmp_str("");
  int tmp_int(0);
  float tmp_float(0.01);
  bool tmp_bool(false);

  if (config.mapGetString(objectName() + "_file", &tmp_str))
    file_->setText(tmp_str);

  if (config.mapGetInt(objectName() + "_number_of_layers", &tmp_int))
    number_of_layers_->setValue(tmp_int);
  else
    number_of_layers_->setValue(1);

  if (config.mapGetFloat(objectName() + "_height_between_layers", &tmp_float))
    height_between_layers_->setValue(tmp_float);
  else
    height_between_layers_->setValue(1);

  if (config.mapGetFloat(objectName() + "_deposited_material_width", &tmp_float))
    deposited_material_width_->setValue(tmp_float);
  else
    deposited_material_width_->setValue(1);

  if (config.mapGetBool(objectName() + "_change_angle", &tmp_bool))
    change_angle_->setChecked(tmp_bool);
  else
    change_angle_->setChecked(false);

  if (config.mapGetFloat(objectName() + "_connection_value", &tmp_float))
    connection_value_->setValue(tmp_float);

  if (config.mapGetInt(objectName() + "_connection_type", &tmp_int))
    connection_type_->setCurrentIndex(tmp_int);
  else
    connection_type_->setCurrentIndex(0);
  Q_EMIT connection_type_->currentIndexChanged(connection_type_->currentIndex());

  if (config.mapGetFloat(objectName() + "_safe_distance", &tmp_float))
    safe_distance_->setValue(tmp_float);
  else
    safe_distance_->setValue(3);

  if (config.mapGetFloat(objectName() + "_offset_factor", &tmp_float))
    offset_factor_->setValue(tmp_float);
  else
    offset_factor_->setValue(0);

  if (config.mapGetBool(objectName() + "_towards_interior", &tmp_bool))
    towards_interior_->setChecked(tmp_bool);
  else
    towards_interior_->setChecked(false);

  if (config.mapGetBool(objectName() + "_reverse_origin_path", &tmp_bool))
    reverse_origin_path_->setChecked(tmp_bool);
  else
    reverse_origin_path_->setChecked(false);

  if (config.mapGetBool(objectName() + "_discontinous_trajectory", &tmp_bool))
    discontinous_trajectory_->setChecked(tmp_bool);
  else
    discontinous_trajectory_->setChecked(false);

  if (config.mapGetBool(objectName() + "_avoid_trajectories_crossing", &tmp_bool))
    avoid_trajectories_crossing_->setChecked(tmp_bool);
  else
    avoid_trajectories_crossing_->setChecked(false);

  if (config.mapGetBool(objectName() + "_automatic_reverse_path", &tmp_bool))
    automatic_reverse_path_->setChecked(tmp_bool);
  else
    automatic_reverse_path_->setChecked(true);

  if (config.mapGetInt(objectName() + "_number_of_passes", &tmp_int))
    number_of_passes_->setValue(tmp_int);
  else
    number_of_passes_->setValue(1);

  if (config.mapGetInt(objectName() + "_join_type", &tmp_int))
    join_type_->setCurrentIndex(tmp_int);

  if (config.mapGetInt(objectName() + "_end_type", &tmp_int))
    end_type_->setCurrentIndex(tmp_int);

  if (config.mapGetFloat(objectName() + "_arc_tolerance", &tmp_float))
    arc_tolerance_->setValue(tmp_float);
  else
    arc_tolerance_->setValue(15);

  if (config.mapGetFloat(objectName() + "_miter_limit", &tmp_float))
    miter_limit_->setValue(tmp_float);
  else
    miter_limit_->setValue(2);

  if (config.mapGetInt(objectName() + "_arc_points", &tmp_int))
    arc_points_->setValue(tmp_int);
  else
    arc_points_->setValue(20);

  joinTypeChanged();
}

void PolygonOffsetsWidget::save(rviz::Config config) const
{
  config.mapSetValue(objectName() + "_file", file_->text());
  config.mapSetValue(objectName() + "_number_of_layers", number_of_layers_->value());
  config.mapSetValue(objectName() + "_height_between_layers", height_between_layers_->value());
  config.mapSetValue(objectName() + "_deposited_material_width", deposited_material_width_->value());
  config.mapSetValue(objectName() + "_connection_type", connection_type_->currentIndex());
  config.mapSetValue(objectName() + "_change_angle", change_angle_->isChecked());
  config.mapSetValue(objectName() + "_connection_value", connection_value_->value());
  config.mapSetValue(objectName() + "_safe_distance", safe_distance_->value());
  config.mapSetValue(objectName() + "_offset_factor", offset_factor_->value());
  config.mapSetValue(objectName() + "_towards_interior", towards_interior_->isChecked());
  config.mapSetValue(objectName() + "_reverse_origin_path", reverse_origin_path_->isChecked());
  config.mapSetValue(objectName() + "_discontinous_trajectory", discontinous_trajectory_->isChecked());
  config.mapSetValue(objectName() + "_avoid_trajectories_crossing", avoid_trajectories_crossing_->isChecked());
  config.mapSetValue(objectName() + "_automatic_reverse_path", automatic_reverse_path_->isChecked());
  config.mapSetValue(objectName() + "_number_of_passes", number_of_passes_->value());
  config.mapSetValue(objectName() + "_join_type", join_type_->currentIndex());
  config.mapSetValue(objectName() + "_end_type", end_type_->currentIndex());
  config.mapSetValue(objectName() + "_arc_tolerance", arc_tolerance_->value());
  config.mapSetValue(objectName() + "_miter_limit", miter_limit_->value());
  config.mapSetValue(objectName() + "_arc_points", arc_points_->value());
}

void PolygonOffsetsWidget::joinTypeChanged()
{
  switch (join_type_->currentIndex())
  {
    case 0: // Square type
    {
      miter_limit_->setEnabled(false);
      arc_tolerance_->setEnabled(end_type_->currentIndex() == 4);
      break;
    }
    case 1: // Round type
    {
      miter_limit_->setEnabled(false);
      arc_tolerance_->setEnabled(true);
      break;
    }
    case 2: // Miter type
    {
      arc_tolerance_->setEnabled(end_type_->currentIndex() == 4);
      miter_limit_->setEnabled(end_type_->currentIndex() == 0 || end_type_->currentIndex() == 1);
      break;
    }
    default:
    {
      arc_tolerance_->setEnabled(true);
      miter_limit_->setEnabled(true);
      break;
    }
  }
}

std::string PolygonOffsetsWidget::fillGoal(ram_path_planning::PolygonOffsetsGoal &goal)
{
  if (file_->text().isEmpty())
    return "File name is not specified.";

  goal.file = file_->text().toStdString();
  goal.number_of_layers = number_of_layers_->value();
  goal.height_between_layers = height_between_layers_->value() / 1000.0; // mm->m
  goal.deposited_material_width = deposited_material_width_->value() / 1000.0; // mm->m

  goal.connection_type = connection_type_->currentIndex(); // mm->m
  if (connection_type_->currentIndex() == 0)
    goal.connection_value = connection_value_->value() * M_PI / 180.0; // deg->rad
  else
    goal.connection_value = connection_value_->value() / 1000.0; // rad

  goal.safe_distance = safe_distance_->value() / 1000.0; // mm->m
  goal.offset_factor = offset_factor_->value() / 1000.0; // mm->m
  goal.number_of_passes = number_of_passes_->value();
  goal.towards_interior = towards_interior_->isChecked();
  goal.reverse_origin_path = reverse_origin_path_->isChecked();
  goal.discontinous_trajectory = discontinous_trajectory_->isChecked();
  goal.avoid_trajectories_crossing = avoid_trajectories_crossing_->isChecked();
  goal.automatic_reverse_path = automatic_reverse_path_->isChecked();
  goal.join_type = join_type_->currentIndex();
  goal.end_type = end_type_->currentIndex();
  goal.arc_tolerance = arc_tolerance_->value();
  goal.miter_limit = miter_limit_->value();
  goal.arc_points = arc_points_->value();
  return "";
}

}
