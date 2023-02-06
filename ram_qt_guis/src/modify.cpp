#include <ram_qt_guis/modify.hpp>

namespace ram_qt_guis
{
Modify::Modify(QWidget *parent) : rviz::Panel(parent),
  selection_mode_(0),
  layer_level_(0),
  is_propagating_(false)
{
  connect(this, &Modify::enable, this, &Modify::setEnabled);
  setObjectName("Modify trajectory");
  setName(objectName());

  layout_ = new QVBoxLayout();
  QWidget *scroll_widget = new QWidget;
  scroll_widget->setLayout(layout_);
  QScrollArea *scroll_area = new QScrollArea;
  scroll_area->setWidget(scroll_widget);
  scroll_area->setWidgetResizable(true);
  scroll_area->setFrameShape(QFrame::NoFrame);
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  main_layout->addWidget(scroll_area);

  changeGUIToSelectionMode();

  connect(this, &Modify::displayMessageBox, this, &Modify::displayMessageBoxHandler);

  // Setup service clients
  update_selection_client_ = nh_.serviceClient<ram_display::UpdateSelection>(
                             "ram/display/update_selection");

  get_trajectory_size_client_ = nh_.serviceClient<ram_utils::GetTrajectorySize>(
                                "ram/information/get_trajectory_size");

  get_number_of_layers_client_ = nh_.serviceClient<ram_utils::GetNumberOfLayersLevels>(
                                 "ram/information/get_number_of_layers_levels");

  get_layer_size_client_ = nh_.serviceClient<ram_utils::GetLayerSize>(
                           "ram/information/get_layer_size");

  get_poses_from_trajectory_client_ = nh_.serviceClient<ram_modify_trajectory::GetPosesFromTrajectory>(
                                      "ram/pose_selector/get_poses_from_trajectory");

  get_poses_from_layers_list_client_ = nh_.serviceClient<ram_modify_trajectory::GetPosesFromLayersList>(
                                       "ram/pose_selector/get_poses_from_layers_list");

  get_poses_from_layer_client_ = nh_.serviceClient<ram_modify_trajectory::GetPosesFromLayer>(
                                 "ram/pose_selector/get_poses_from_layer");

  get_poses_from_information_ = nh_.serviceClient<ram_modify_trajectory::GetPosesFromInformation>(
                                "ram/pose_selector/get_poses_from_information");

  // Modify services
  modify_selected_poses_client_ = nh_.serviceClient<ram_modify_trajectory::ModifySelectedPoses>(
                                  "ram/modify_trajectory/modify_selected_poses");

  delete_selected_poses_client_ = nh_.serviceClient<ram_modify_trajectory::DeleteSelectedPoses>(
                                  "ram/modify_trajectory/delete_selected_poses");

  add_poses_client_ = nh_.serviceClient<ram_modify_trajectory::AddPoses>("ram/modify_trajectory/add_poses");

  reset_selected_poses_client_ = nh_.serviceClient<ram_modify_trajectory::ResetSelectedPoses>(
                                 "ram/modify_trajectory/reset_selected_poses");

  trajectory_interruption_client_ = nh_.serviceClient<ram_modify_trajectory::TrajectoryInterruption>(
                                    "ram/modify_trajectory/trajectory_interruption");

  change_layer_height_client_ = nh_.serviceClient<ram_modify_trajectory::ChangeLayerHeight>(
                                "ram/modify_trajectory/change_layer_height");

  tweak_blend_radius_client_ = nh_.serviceClient<ram_modify_trajectory::TweakBlendRadius>(
                               "ram/modify_trajectory/tweak_blend_radius");

  simplify_trajectory_client_ = nh_.serviceClient<ram_modify_trajectory::SimplifyTrajectory>(
                               "ram/modify_trajectory/simplify_trajectory");

  push_pull_angle_client_ = nh_.serviceClient<ram_modify_trajectory::PushPullAngle>(
                               "ram/modify_trajectory/push_pull_angle");

  // Geometric operation services
  rotate_selected_poses_client_ = nh_.serviceClient<ram_modify_trajectory::RotateSelectedPoses>(
                                  "ram/modify_trajectory/rotate_selected_poses");

  reflect_selected_poses_client_ = nh_.serviceClient<ram_modify_trajectory::ReflectSelectedPoses>(
                                   "ram/modify_trajectory/reflect_selected_poses");

  scale_selected_poses_client_ = nh_.serviceClient<ram_modify_trajectory::ScaleSelectedPoses>(
                                 "ram/modify_trajectory/scale_selected_poses");

  shift_poses_client_ = nh_.serviceClient<ram_modify_trajectory::ShiftPoses>(
                        "ram/modify_trajectory/shift_poses");

  // Trajectory
  traj_ = nh_.subscribe("ram/trajectory", 10, &Modify::trajReceived, this);

  // Check connection of client
  QtConcurrent::run(this, &Modify::connectToServices);
}

Modify::~Modify()
{
}

void Modify::trajReceived(const ram_msgs::AdditiveManufacturingTrajectory::Ptr &msg)
{
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  trajectory_ = *msg;

  changeGUIToSelectionMode();
}

void Modify::clearLayout(QLayout *layout,
                         bool delete_widgets)
{
  const bool old_state(isEnabled());
  Q_EMIT enable(false);

  while (QLayoutItem *item = layout->takeAt(0))
  {
    QWidget *widget;
    if ((delete_widgets)
        && (widget = item->widget()))
    {
      delete widget;
    }
    if (QLayout *childLayout = item->layout())
    {
      clearLayout(childLayout, delete_widgets);
    }
    delete item;
  }

  Q_EMIT enable(old_state);
}

void Modify::changeGUIToSelectionMode()
{
  Q_EMIT enable(false);

  clearLayout(layout_);
  layers_to_propagate_.clear();
  is_propagating_ = false;

  // Clear displayed selection
  ram_display::UpdateSelection srv; // Empty
  update_selection_client_.call(srv);

  QGridLayout *buttons = new QGridLayout;

  selection_buttons_.clear();
  selection_buttons_.push_back(new QRadioButton(""));
  selection_buttons_.push_back(new QRadioButton(""));
  selection_buttons_.push_back(new QRadioButton(""));
  selection_buttons_.push_back(new QRadioButton(""));
  selection_buttons_.push_back(new QRadioButton(""));

  buttons->addWidget(selection_buttons_.at(0), 0, 0);
  QLabel *label_1 = new QLabel("Whole trajectory\nSelect all the poses of the trajectory\n");
  label_1->setWordWrap(true);
  buttons->addWidget(label_1, 0, 1);

  buttons->addWidget(selection_buttons_.at(1));
  QLabel *label_2 = new QLabel("Trajectory\nSelect one pose or more within the trajectory\n");
  label_2->setWordWrap(true);
  buttons->addWidget(label_2);

  buttons->addWidget(selection_buttons_.at(2));
  QLabel *label_3 = new QLabel("Layers\nSelect one layer or more\n");
  label_3->setWordWrap(true);
  buttons->addWidget(label_3);

  buttons->addWidget(selection_buttons_.at(3));
  QLabel *label_4 = new QLabel("Within layer\nSelect one pose or more within a specific layer.\n"
                               "You will be able to propagate this selection across layers afterwards.\n");
  label_4->setWordWrap(true);
  buttons->addWidget(label_4);

  buttons->addWidget(selection_buttons_.at(4));
  QLabel *label_5 = new QLabel("Filter by information\nWith poses information and/or parameters\n"
                               "Select poses with their characteristics (Entry, Exit pose...)\n");
  label_5->setWordWrap(true);
  buttons->addWidget(label_5);

  QPushButton *start_selection = new QPushButton("Start selection");

  layout_->addWidget(new QLabel("Selection mode:"));
  layout_->addLayout(buttons);

  selection_buttons_.at(selection_mode_)->setChecked(true);

  layout_->addStretch(1);
  layout_->addWidget(start_selection);

  connect(start_selection, &QPushButton::clicked, this, &Modify::selectionModeSelected);

  Q_EMIT enable(true);
}

void Modify::selectionModeSelected()
{
  // Get selection mode
  unsigned selected_button(0);
  for (auto button : selection_buttons_)
  {
    if (button->isChecked())
      break;

    ++selected_button;
  }
  selection_mode_ = selected_button;
  Q_EMIT configChanged();

  QString help_string;
  unsigned min_value(0), max_value(1);

  // Whole trajectory
  if (selection_mode_ == 0)
  {
    ram_utils::GetNumberOfLayersLevels srv_1;
    if (!get_number_of_layers_client_.call(srv_1) || !srv_1.response.error.empty())
    {
      Q_EMIT displayMessageBox("Modify trajectory - Selection mode",
                               "Could not get the number of layers",
                               QString::fromStdString(srv_1.response.error),
                               QMessageBox::Icon::Critical);

      Q_EMIT enable(true);
      return;
    }

    if (srv_1.response.number_of_layers == 0)
    {
      Q_EMIT displayMessageBox("Modify trajectory - Selection mode",
                               "The trajectory contains zero layer",
                               "The number of layer is 0 (the trajectory is probably empty)",
                               QMessageBox::Icon::Critical);

      Q_EMIT enable(true);
      return;
    }

    ram_modify_trajectory::GetPosesFromLayersList srv_2;
    std::vector<unsigned> layers;
    for (std::size_t i(0); i < srv_1.response.number_of_layers; ++i)
      layers.emplace_back(i);
    srv_2.request.layer_level_list = layers;
    if (!get_poses_from_layers_list_client_.call(srv_2) || !srv_2.response.error.empty())
    {
      Q_EMIT displayMessageBox("Selection",
                                "Could not get poses from the layers list indices :",
                                QString::fromStdString(srv_2.response.error),
                                QMessageBox::Icon::Critical);
      Q_EMIT enable(true);
      return;
    }

    selected_poses_ = srv_2.response.poses;
    if (selected_poses_.empty())
    {
      Q_EMIT displayMessageBox("Selection",
                                "The selection is empty, cannot continue : ",
                                QString::fromStdString(srv_2.response.error),
                                QMessageBox::Icon::Critical);
      Q_EMIT enable(true);
      return;
    }

    changeGUIToOperationSelection();
    return;
  }

  // Trajectory selection
  else if (selection_mode_ == 1)
  {
    ram_utils::GetTrajectorySize srv;
    if (!get_trajectory_size_client_.call(srv))
    {
      Q_EMIT displayMessageBox("Modify trajectory - Selection mode",
                               "Could not get the trajectory size",
                               "",
                               QMessageBox::Icon::Critical);

      changeGUIToSelectionMode();
      return;
    }

    if (srv.response.trajectory_size == 0)
    {
      Q_EMIT displayMessageBox("Modify trajectory - Selection mode",
                               "The trajectory is empty",
                               "Generate a trajectory first",
                               QMessageBox::Icon::Critical);

      changeGUIToSelectionMode();
      return;
    }

    help_string = "Pick up the poses to be selected:";
    max_value = srv.response.trajectory_size - 1;
  }

  // Layers selection OR within layer
  else if (selection_mode_ == 2 || selection_mode_ == 3)
  {
    ram_utils::GetNumberOfLayersLevels srv;
    if (!get_number_of_layers_client_.call(srv) || !srv.response.error.empty())
    {
      Q_EMIT displayMessageBox("Modify trajectory - Selection mode",
                               "Could not get the number of layers",
                               QString::fromStdString(srv.response.error),
                               QMessageBox::Icon::Critical);

      changeGUIToSelectionMode();
      return;
    }

    if (srv.response.number_of_layers == 0)
    {
      Q_EMIT displayMessageBox("Modify trajectory - Selection mode",
                               "The trajectory contains zero layer",
                               "The number of layer is 0 (the trajectory is probably empty)",
                               QMessageBox::Icon::Critical);

      changeGUIToSelectionMode();
      return;
    }

    if (selection_mode_ == 2)
    {
      help_string = "Pick up the layers to be selected:";
      max_value = srv.response.number_of_layers - 1;
    }
    else if (selection_mode_ == 3)
    {
      // Ask user which layer we are going to work with
      QDialog *window = new QDialog;
      window->setWindowTitle("Modify trajectory - Selection");
      window->setModal(true);
      QVBoxLayout *window_layout = new QVBoxLayout(window);
      window->setLayout(window_layout);
      window_layout->addWidget(new QLabel("Pick a layer to work with:"));
      QSpinBox *layers_list = new QSpinBox;
      layers_list->setRange(0, srv.response.number_of_layers - 1);
      window_layout->addWidget(layers_list);

      QDialogButtonBox *button_box = new QDialogButtonBox(QDialogButtonBox::Ok
          | QDialogButtonBox::Cancel);
      window_layout->addStretch(1);
      window_layout->addWidget(button_box);
      connect(button_box, &QDialogButtonBox::accepted, window, &QDialog::accept);
      connect(button_box, &QDialogButtonBox::rejected, window, &QDialog::reject);

      if (!window->exec())
      {
        changeGUIToSelectionMode();
        return;
      }
      layer_level_ = layers_list->value();

      ram_utils::GetLayerSize srv;
      srv.request.layer_level = layer_level_;
      if (!get_layer_size_client_.call(srv) || !srv.response.error.empty())
      {
        Q_EMIT displayMessageBox("Modify trajectory - Selection mode",
                                 "Could not get the layer " + QString::number(layer_level_) + " size",
                                 QString::fromStdString(srv.response.error),
                                 QMessageBox::Icon::Critical);

        changeGUIToSelectionMode();
        return;
      }

      if (srv.response.layer_size == 0)
      {
        Q_EMIT displayMessageBox("Modify trajectory - Selection mode",
                                 "Could not get the layer " + QString::number(layer_level_) + " size",
                                 "The layer size is 0 (It is probably empty!)",
                                 QMessageBox::Icon::Critical);
        return;
      }

      help_string = "Pick up the poses to be selected inside layer " + QString::number(layer_level_) + ":";
      max_value = srv.response.layer_size - 1;
    }
  }

  // Poses selection with information and/or parameters
  else if (selection_mode_ == 4)
  {
    ram_utils::GetTrajectorySize srv_1;
    if (!get_trajectory_size_client_.call(srv_1) || (srv_1.response.trajectory_size == 0))
    {
      Q_EMIT displayMessageBox("Modify trajectory - Selection mode",
                               "Could not continue : \nThe trajectory is empty",
                               "",
                               QMessageBox::Icon::Critical);

      Q_EMIT enable(true);
      return;
    }
    else
    {
      help_string = "Choose the option to select and filter these " + QString::number(srv_1.response.trajectory_size) +
                    " poses :\n";
      changeGUIToInformationFilterSelection(help_string);
      return;
    }
  }

  // Error
  else
  {
    Q_EMIT displayMessageBox("Modify trajectory - Selection mode",
                             "Selection mode is out of range!",
                             "",
                             QMessageBox::Icon::Critical);
    changeGUIToSelectionMode();
    return;
  }

  changeGUIToRangeListSelection(help_string, min_value, max_value);
}

void Modify::changeGUIToInformationFilterSelection(const QString help_string)
{
  Q_EMIT enable(false);
  clearLayout(layout_);

  information_filter_selection_ui_ = new PoseInformationParametersFilter(layout_, help_string);

  connect(information_filter_selection_ui_->button_box_, &QDialogButtonBox::accepted, this,
          &Modify::getSelectionFromInformationFilter);
  connect(information_filter_selection_ui_->button_box_, &QDialogButtonBox::rejected, this,
          &Modify::changeGUIToSelectionMode);
  Q_EMIT enable(true);
}

void Modify::getSelectionFromInformationFilter()
{
  Q_EMIT enable(false);
  ram_modify_trajectory::GetPosesFromInformation srv;

  srv.request.information_parameters.polygon_start = information_filter_selection_ui_->getStartStatus();
  srv.request.information_parameters.polygon_end = information_filter_selection_ui_->getEndStatus();
  srv.request.information_parameters.both_start_end = information_filter_selection_ui_->getStartEndStatus();
  srv.request.information_parameters.last_pose_in_layer = information_filter_selection_ui_->getLastPoseInLayerStatus();

  if (!get_poses_from_information_.call(srv) || !srv.response.error.empty())
  {
    Q_EMIT displayMessageBox("Selection",
                             "Could not get poses from the previous filters selection :",
                             QString::fromStdString(srv.response.error),
                             QMessageBox::Icon::Critical);
    return;
  }
  else
  {
    Q_EMIT enable(true);
    selected_poses_ = srv.response.selection;
    changeGUIToOperationSelection();
  }
}

void Modify::changeGUIToRangeListSelection(const QString help_string, const unsigned min, const unsigned max)
{
  Q_EMIT enable(false);

  clearLayout(layout_);
  range_list_selection_ui_ = new ModifyRangeListSelection(layout_, help_string, min, max);

  connect(range_list_selection_ui_, &ModifyRangeListSelection::selectionChanged, this,
          &Modify::updateTemporarySelection);
  connect(range_list_selection_ui_->button_box_, &QDialogButtonBox::accepted, this, &Modify::getSelection);
  connect(range_list_selection_ui_->button_box_, &QDialogButtonBox::rejected, this, &Modify::changeGUIToSelectionMode);

  Q_EMIT enable(true);
}

void Modify::getSelection()
{
  std::vector<unsigned> selected(range_list_selection_ui_->getSelection());
  if (selected.empty())
  {
    Q_EMIT displayMessageBox("Empty selection",
                             "Cannot continue, the selection is empty!",
                             "",
                             QMessageBox::Icon::Critical);
    return;
  }

  Q_EMIT enable(false);

  // Make a service request to get the selection
  switch (selection_mode_)
  {
    case 0:
    {
    }
    case 1:
      {
        ram_modify_trajectory::GetPosesFromTrajectory srv;
        srv.request.pose_index_list = selected;
        if (!get_poses_from_trajectory_client_.call(srv) || srv.response.poses.empty() || !srv.response.error.empty())
        {
          Q_EMIT displayMessageBox("Selection",
                                   "Could not get poses from the trajectory poses indices :",
                                   QString::fromStdString(srv.response.error),
                                   QMessageBox::Icon::Critical);
          Q_EMIT enable(true);
          return;
        }

        selected_poses_ = srv.response.poses;
        break;
      }
    case 2:
      {
        ram_modify_trajectory::GetPosesFromLayersList srv;
        srv.request.layer_level_list = selected;
        if (!get_poses_from_layers_list_client_.call(srv) || !srv.response.error.empty())
        {
          Q_EMIT displayMessageBox("Selection",
                                   "Could not get poses from the layers list indices :",
                                   QString::fromStdString(srv.response.error),
                                   QMessageBox::Icon::Critical);
          Q_EMIT enable(true);
          return;
        }

        selected_poses_ = srv.response.poses;
        if (selected_poses_.empty())
        {
          Q_EMIT displayMessageBox("Selection",
                                   "The selection is empty, cannot continue : ",
                                   QString::fromStdString(srv.response.error),
                                   QMessageBox::Icon::Critical);
          Q_EMIT enable(true);
          return;
        }
        break;
      }
    case 3:
      {
        ram_modify_trajectory::GetPosesFromLayer srv;
        srv.request.layer_level = layer_level_;
        srv.request.index_list_relative = selected;
        if (!get_poses_from_layer_client_.call(srv) || !srv.response.error.empty())
        {
          Q_EMIT displayMessageBox("Selection",
                                   "Could not get poses from the layer relative indices list :",
                                   QString::fromStdString(srv.response.error),
                                   QMessageBox::Icon::Critical);
          Q_EMIT enable(true);
          return;
        }
      relative_indices_ = selected;
      break;
    }
    default:
      {
        Q_EMIT displayMessageBox("Selection mode not recognized",
                                 "Missing implementation in getSelection()",
                                 "",
                                 QMessageBox::Icon::Critical);
        clearLayout(layout_);
        Q_EMIT enable(true);
        changeGUIToSelectionMode();
        return;
      }
  }

  Q_EMIT enable(true);

  // Propagate selection across layers
  if (selection_mode_ == 3)
  {
    ram_utils::GetNumberOfLayersLevels srv;
    if (!get_number_of_layers_client_.call(srv) || !srv.response.error.empty())
    {
      Q_EMIT displayMessageBox("Modify trajectory - Propagate",
                               "Could not get the number of layers",
                               QString::fromStdString(srv.response.error),
                               QMessageBox::Icon::Critical);

      changeGUIToSelectionMode();
      return;
    }

    Q_EMIT enable(false);
    clearLayout(layout_);
    std::vector<unsigned> locked;
    locked.push_back(layer_level_);

    range_list_selection_ui_ = new ModifyRangeListSelection(layout_, "Propagate to layers:", 0,
        srv.response.number_of_layers - 1,
        locked);

    is_propagating_ = true;
    connect(range_list_selection_ui_, &ModifyRangeListSelection::selectionChanged, this,
            &Modify::updateTemporarySelection);
    connect(range_list_selection_ui_->button_box_, &QDialogButtonBox::accepted, this, &Modify::propagateSelection);
    connect(range_list_selection_ui_->button_box_, &QDialogButtonBox::rejected, this,
            &Modify::changeGUIToSelectionMode);

    Q_EMIT enable(true);
  }
  else
    changeGUIToOperationSelection();
}

void Modify::propagateSelection()
{
  layers_to_propagate_ = range_list_selection_ui_->getSelection();
  if (layers_to_propagate_.empty())
  {
    Q_EMIT displayMessageBox("Propagate selection",
                             "Layers to propagate vector is empty!",
                             "",
                             QMessageBox::Icon::Critical);
    changeGUIToSelectionMode();
    return;
  }

  selected_poses_.clear();

  // Call GetPosesFromLayer multiple times and check whether this is ok
  for (auto &layer : layers_to_propagate_)
  {
    ram_modify_trajectory::GetPosesFromLayer srv;
    srv.request.index_list_relative = relative_indices_;
    srv.request.layer_level = layer;
    if (!get_poses_from_layer_client_.call(srv) || !srv.response.error.empty())
    {
      Q_EMIT displayMessageBox("Propagate selection",
                               QString::fromStdString(
                               "Failed to propagate selection to layer " + std::to_string(layer) + "\n" +
                               "Error : " +
                               srv.response.error),
                               "De-select this layer and try again.",
                               QMessageBox::Icon::Critical);
      return;
    }

    // Append to selection vector
    selected_poses_.insert(selected_poses_.end(), srv.response.poses.begin(), srv.response.poses.end());
  }

  is_propagating_ = false;
  changeGUIToOperationSelection();
}

void Modify::changeGUIToOperationSelection()
{
  Q_EMIT enable(false);
  clearLayout(layout_);

  ram_display::UpdateSelection srv;
  srv.request.selected_poses = selected_poses_;
  srv.request.temporary = false;
  if (!update_selection_client_.call(srv))
  {
    Q_EMIT displayMessageBox("Failed to display selection",
                             "The selection cannot be displayed",
                             "Aborting operation",
                             QMessageBox::Icon::Critical);
    changeGUIToSelectionMode();
  }

  QLabel *list_title = new QLabel;
  list_title->setWordWrap(true);
  if (selected_poses_.size() > 1)
    list_title->setText(QString::fromStdString(std::to_string(selected_poses_.size()) + " poses are selected. "));
  else
    list_title->setText(QString::fromStdString(std::to_string(selected_poses_.size()) + " pose is selected. "));

  std::string layers_list;
  for (auto &pose : selected_poses_)
  {
    if (layers_list.empty())
      layers_list += std::to_string(pose.layer_index);
    if (layers_list.find(std::to_string(pose.layer_index)) == std::string::npos)
      layers_list += ", " + std::to_string(pose.layer_index);
  }

  if (layers_list.size() > 1)
    list_title->setText(list_title->text() + "\nLayers " + QString::fromStdString(layers_list) + " are selected.");
  else
    list_title->setText(list_title->text() + "\nLayer " + QString::fromStdString(layers_list) + " is selected.");

  layout_->addWidget(list_title);
  layout_->addStretch(1);
  layout_->addWidget(new QLabel("<b>Operation:</b>"));

  QRadioButton *button_0 = new QRadioButton("Modify");
  QRadioButton *button_1 = new QRadioButton("Add");
  QRadioButton *button_2 = new QRadioButton("Delete");
  QRadioButton *button_3 = new QRadioButton("Reset");
  QRadioButton *button_4 = new QRadioButton("Interruption");
  QRadioButton *button_5 = new QRadioButton("Height between layers");
  QRadioButton *button_6 = new QRadioButton("Tweak blend radius");
  QRadioButton *button_7 = new QRadioButton("Simplify");
  QRadioButton *button_8 = new QRadioButton("Push/pull angle");

  operations_.clear();
  operations_.push_back(button_0);
  operations_.push_back(button_1);
  operations_.push_back(button_2);
  operations_.push_back(button_3);
  operations_.push_back(button_4);
  operations_.push_back(button_5);
  operations_.push_back(button_6);
  operations_.push_back(button_7);
  operations_.push_back(button_8);

  button_0->setChecked(true);

  for (auto button : operations_)
    layout_->addWidget(button);

  layout_->addStretch(1);
  layout_->addWidget(new QLabel("<b>Geometric operation:</b>"));

  QRadioButton *button_a = new QRadioButton("Rotate");
  QRadioButton *button_b = new QRadioButton("Reflect");
  QRadioButton *button_c = new QRadioButton("Scale");
  QRadioButton *button_d = new QRadioButton("Shift");

  geometric_operations_.clear();
  geometric_operations_.push_back(button_a);
  geometric_operations_.push_back(button_b);
  geometric_operations_.push_back(button_c);
  geometric_operations_.push_back(button_d);

  for (auto button : geometric_operations_)
    layout_->addWidget(button);

  QDialogButtonBox *button_box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  layout_->addStretch(2);
  layout_->addWidget(button_box);

  connect(button_box, &QDialogButtonBox::accepted, this, &Modify::operationSelected);
  connect(button_box, &QDialogButtonBox::rejected, this, &Modify::changeGUIToSelectionMode);

  Q_EMIT enable(true);
}

void Modify::operationSelected()
{
  unsigned operation_mode(0);
  for (auto button : operations_)
    if (button && button->isChecked())
      break;
    else
      ++operation_mode;

  if (operation_mode == operations_.size())
  {
    for (auto button : geometric_operations_)
      if (button && button->isChecked())
        break;
      else
        ++operation_mode;
  }

  Q_EMIT enable(false);

  switch (operation_mode)
  {
    // Modify poses
    case 0:
      {
        changeGUIToModifyPoses();
        return;
      }
    // Add poses
    case 1:
      {
        ram_modify_trajectory::AddPoses srv;
        srv.request.poses = selected_poses_;
        if (!add_poses_client_.call(srv) || !srv.response.error.empty())
        {
          Q_EMIT displayMessageBox("Error",
                                   "Could not add poses!",
                                   "Pick another one." + QString::fromStdString("\nError : " + srv.response.error),
                                   QMessageBox::Icon::Critical);
          Q_EMIT enable(true);
          return;
        }
        break;
      }
    // Delete poses
    case 2:
      {
        ram_modify_trajectory::DeleteSelectedPoses srv;
        srv.request.poses = selected_poses_;
        if (!delete_selected_poses_client_.call(srv) || !srv.response.error.empty())
        {
          Q_EMIT displayMessageBox("Error",
                                   "Could not delete the selection!",
                                   "Pick another one. Note that you cannot delete the whole trajectory"
                                   + QString::fromStdString("\nError : " + srv.response.error),
                                   QMessageBox::Icon::Critical);
          Q_EMIT enable(true);
          return;
        }
        break;
      }
    // Reset poses
    case 3:
      {
        ram_modify_trajectory::ResetSelectedPoses srv;
        srv.request.poses = selected_poses_;
        if (!reset_selected_poses_client_.call(srv) || !srv.response.error.empty())
        {
          Q_EMIT displayMessageBox("Error",
                                   "Could not reset the selection!",
                                   "Pick another one." + QString::fromStdString("\nError : " + srv.response.error),
                                   QMessageBox::Icon::Critical);
          Q_EMIT enable(true);
          return;
        }
        break;
      }
    // Trajectory interruption
    case 4:
      {
        changeGUIToTrajectoryInterruption();
        return;
      }
    case 5:
      {
        changeGUIToHeightModification();
        return;
      }
    // Tweak blend radius on selected poses
    case 6:
      {
        changeGUIToTweakBlendRadius();
        return;
      }
    // Simplify trajectory
    case 7:
      {
        changeGUIToSimplifyTrajectory();
        return;
      }
    // Push/pull angle
    case 8:
      {
        changeGUIToPushPullAngle();
        return;
      }
    // Rotate poses
    case 9:
      {
        changeGUIToRotatePoses();
        return;
      }
    // Reflect poses
    case 10:
      {
        changeGUIToReflectPoses();
        return;
      }
    // Dilate poses
    case 11:
      {
        changeGUIToScalePoses();
        return;
      }
    // Shift poses
    case 12:
      {
        changeGUIToShiftPoses();
        return;
      }
    default:
      Q_EMIT displayMessageBox("Operation",
                               "Error selecting operation mode",
                               "",
                               QMessageBox::Icon::Critical);
      return;
  }

  Q_EMIT enable(true);
  changeGUIToSelectionMode();
}

void Modify::changeGUIToModifyPoses()
{
  Q_EMIT enable(false);
  clearLayout(layout_);

  // Determine if all poses have the same motion type or not
  // If yes, modifying speed is enabled
  // If no, modifying speed is only enabled if movement type is modified
  if (selected_poses_.empty())
  {
    clearLayout(layout_);
    Q_EMIT displayMessageBox("Modify poses",
                             "Empty selection, aborting.",
                             "",
                             QMessageBox::Icon::Critical);
    Q_EMIT enable(true);
    return;
  }

  std::vector<ram_msgs::AdditiveManufacturingParams::_movement_type_type> move_types;
  for (auto pose : selected_poses_)
    move_types.push_back(pose.params.movement_type);

  if (std::all_of(move_types.cbegin(), move_types.cend(), [](int i)
{
  return i == 0;
}))
  {
    modify_poses_ui_ = new ModifyPoses(layout_, 0);
  }
  else if (std::all_of(move_types.cbegin(), move_types.cend(), [](int i)
{
  return i == 1;
}))
  {
    modify_poses_ui_ = new ModifyPoses(layout_, 1);
  }
  else
    modify_poses_ui_ = new ModifyPoses(layout_, 2);

  connect(modify_poses_ui_, &ModifyPoses::enable, this, &ModifyPoses::setEnabled);
  connect(modify_poses_ui_->button_box_, &QDialogButtonBox::accepted, this, &Modify::modifyPoses);
  connect(modify_poses_ui_->button_box_, &QDialogButtonBox::rejected, this, &Modify::changeGUIToSelectionMode);

  Q_EMIT enable(true);
}

void Modify::changeGUIToTrajectoryInterruption()
{
  Q_EMIT enable(false);
  clearLayout(layout_);

  traj_interruption_add_pose_ = new QCheckBox("Add poses");
  layout_->addWidget(traj_interruption_add_pose_);
  layout_->addStretch(1);
  QLabel *description(new QLabel);
  description->setText(("If checked the trajectory will start where it stopped.\n\n"
    "Otherwise the trajectory will stop and start at the next point in the trajectory."));
  description->setWordWrap(true);
  layout_->addWidget(description);
  layout_->addStretch(2);

  QDialogButtonBox *button_box = new QDialogButtonBox(QDialogButtonBox::Ok
      | QDialogButtonBox::Cancel);
  layout_->addStretch(2);
  layout_->addWidget(button_box);

  connect(button_box, &QDialogButtonBox::accepted, this, &Modify::trajectoryInterruption);
  connect(button_box, &QDialogButtonBox::rejected, this, &Modify::changeGUIToSelectionMode);

  Q_EMIT enable(true);
}

void Modify::trajectoryInterruption()
{
  ram_modify_trajectory::TrajectoryInterruption srv;
  srv.request.poses = selected_poses_;
  srv.request.add_pose = traj_interruption_add_pose_->isChecked();
  if (!trajectory_interruption_client_.call(srv) || !srv.response.error.empty())
  {
    Q_EMIT displayMessageBox("Error",
                             "Could not add trajectory interruptions to the selection!",
                             QString::fromStdString("Error : " + srv.response.error),
                             QMessageBox::Icon::Critical);
  }
  changeGUIToSelectionMode();
}

void Modify::changeGUIToTweakBlendRadius()
{
  Q_EMIT enable(false);
  clearLayout(layout_);

  QLabel *label(new QLabel("Entry/exit, polygon start/end poses will not be modified"));
  label->setWordWrap(true);
  layout_->addWidget(label);

  blend_radius_table_ = new BlendRadiusTable;
  layout_->addWidget(blend_radius_table_);

  QDialogButtonBox *button_box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  layout_->addWidget(button_box);

  connect(button_box, &QDialogButtonBox::accepted, this, [ = ]()
  {
    ram_modify_trajectory::TweakBlendRadius srv;
    srv.request.poses = selected_poses_;
    blend_radius_table_->fillRequest(srv.request);

    if (!tweak_blend_radius_client_.call(srv) || !srv.response.error.empty())
      Q_EMIT displayMessageBox("Error",
                               "Cannot tweak the blend radius of this selection!",
                               QString::fromStdString(srv.response.error),
                               QMessageBox::Icon::Critical);
    else
      changeGUIToSelectionMode();
  });
  connect(button_box, &QDialogButtonBox::rejected, this, &Modify::changeGUIToSelectionMode);

  Q_EMIT enable(true);
}

void Modify::changeGUIToSimplifyTrajectory()
{
  Q_EMIT enable(false);
  clearLayout(layout_);

  QLabel *label(new QLabel("Simplify trajectory by deleting smallest vertices\n"
                           "Entry/exit, polygon start/end will not be modified"));
  label->setWordWrap(true);

  QWidget* simplify_trajectory_threshold_widget = new QWidget;
  QHBoxLayout *simplify_trajectory_threshold_layout = new QHBoxLayout;
  simplify_trajectory_threshold_widget->setLayout(simplify_trajectory_threshold_layout);
  QLabel *simplify_trajectory_threshold_label = new QLabel("Area threshold");
  simplify_trajectory_threshold_layout->addWidget(simplify_trajectory_threshold_label);

  simplify_trajectory_threshold_ = new QDoubleSpinBox(simplify_trajectory_threshold_label);
  simplify_trajectory_threshold_->setToolTip("Set the surface below whose the vertices will be deleted");
  simplify_trajectory_threshold_->setSuffix(" µm²");
  simplify_trajectory_threshold_->setRange(0.1, 100000);
  simplify_trajectory_threshold_->setDecimals(1);
  simplify_trajectory_threshold_->setSingleStep(50);
  simplify_trajectory_threshold_->setValue(100);
  simplify_trajectory_threshold_layout->addWidget(simplify_trajectory_threshold_);

  layout_->addWidget(label);
  layout_->addStretch(1);
  layout_->addWidget(simplify_trajectory_threshold_widget);

  QLabel *equilateral_equivalent_triangle(new QLabel);
  layout_->addWidget(equilateral_equivalent_triangle);

  connect(simplify_trajectory_threshold_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [=]()
  {
    const double length(sqrt(4*simplify_trajectory_threshold_->value()/sqrt(3)));
    equilateral_equivalent_triangle->setText("Equilateral triangle length: " + QString::number(length, 'g', 3) + " µm");
  });
  Q_EMIT simplify_trajectory_threshold_->valueChanged(simplify_trajectory_threshold_->value());

  QDialogButtonBox *button_box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  layout_->addWidget(button_box);

  connect(button_box, &QDialogButtonBox::accepted, this, [&]()
  {
    ram_modify_trajectory::SimplifyTrajectory srv;
    srv.request.poses = selected_poses_;
    srv.request.area_threshold = simplify_trajectory_threshold_->value()/1e12; // in m²

    if (!simplify_trajectory_client_.call(srv) || !srv.response.error.empty())
      Q_EMIT displayMessageBox("Error",
                               "Cannot simplify the trajectory of this selection!",
                               QString::fromStdString(srv.response.error),
                               QMessageBox::Icon::Critical);
    else
      changeGUIToSelectionMode();
  });
  connect(button_box, &QDialogButtonBox::rejected, this, &Modify::changeGUIToSelectionMode);

  Q_EMIT enable(true);
}

void Modify::changeGUIToPushPullAngle()
{
  Q_EMIT enable(false);
  clearLayout(layout_);

  push_pull_angle_ = new QDoubleSpinBox;
  push_pull_angle_->setSuffix(" deg");
  push_pull_angle_->setRange(-90, 90);
  push_pull_angle_->setSingleStep(1);
  push_pull_angle_->setValue(35);

  QWidget* push_pull_angle_widget(new QWidget);
  QHBoxLayout *push_pull_angle_layout(new QHBoxLayout(push_pull_angle_widget));
  push_pull_angle_layout->addWidget(new QLabel("Angle"));
  push_pull_angle_layout->addWidget(push_pull_angle_);

  layout_->addWidget(push_pull_angle_widget);

  QDialogButtonBox *button_box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  layout_->addWidget(button_box);

  connect(button_box, &QDialogButtonBox::accepted, this, [&]()
  {
    ram_modify_trajectory::PushPullAngle srv;
    srv.request.poses = selected_poses_;
    srv.request.angle = push_pull_angle_->value() * M_PI / 180.0; // degres to radians

    if (!push_pull_angle_client_.call(srv) || !srv.response.error.empty())
      Q_EMIT displayMessageBox("Error",
                               "Service push/pull failed on this selection!",
                               QString::fromStdString(srv.response.error),
                               QMessageBox::Icon::Critical);
    else
      changeGUIToSelectionMode();
  });
  connect(button_box, &QDialogButtonBox::rejected, this, &Modify::changeGUIToSelectionMode);
  Q_EMIT enable(true);
}

void Modify::changeGUIToHeightModification()
{
  Q_EMIT enable(false);
  clearLayout(layout_);

  mode_ = new QComboBox;
  mode_->addItem("Relative mode");
  mode_->addItem("Absolute mode");
  mode_->setToolTip("- Absolute mode : the requested height replaces the current height\n"
                    "- Relative mode : the requested value is added to the current height (according to its sign)");
  QLabel *height_label = new QLabel("Layer height");

  new_height_ = new QDoubleSpinBox(height_label);
  new_height_->setRange(-100, 100);
  new_height_->setToolTip(height_label->toolTip());
  new_height_->setSuffix(" mm");

  connect(mode_, qOverload<int>(&QComboBox::currentIndexChanged), this, [ = ](int index)
  {
    if (index == 0)
      new_height_->setRange(-100, 100);
    else
      new_height_->setRange(0.01, 100);
  });
  mode_->setCurrentIndex(0);

  QHBoxLayout *height_layout = new QHBoxLayout();
  height_layout->addWidget(height_label);
  height_layout->addWidget(new_height_);
  layout_->addWidget(mode_);
  layout_->addLayout(height_layout);
  layout_->addStretch(1);

  QDialogButtonBox *button_box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  layout_->addStretch(2);
  layout_->addWidget(button_box);

  connect(button_box, &QDialogButtonBox::accepted, this, &Modify::heightModification);
  connect(button_box, &QDialogButtonBox::rejected, this, &Modify::changeGUIToSelectionMode);

  Q_EMIT enable(true);
}

void Modify::heightModification()
{
  ram_modify_trajectory::ChangeLayerHeight srv;

  srv.request.poses = selected_poses_;
  srv.request.height = (new_height_->value()) / 1000; // to meters
  srv.request.absolute_mode = mode_->currentIndex();

  if (!change_layer_height_client_.call(srv) || !srv.response.error.empty())
    Q_EMIT displayMessageBox("Error",
                             "Cannot change the layer height of this selection!",
                             QString::fromStdString(srv.response.error),
                             QMessageBox::Icon::Critical);

  changeGUIToSelectionMode();
}

void Modify::modifyPoses()
{
  // Check that at least one tick is checked!
  std::vector<QCheckBox *> checkboxes;
  checkboxes.push_back(modify_poses_ui_->approach_type_modify_);
  checkboxes.push_back(modify_poses_ui_->blend_radius_modify_);
  checkboxes.push_back(modify_poses_ui_->feed_rate_modify_);
  checkboxes.push_back(modify_poses_ui_->laser_power_modify_);
  checkboxes.push_back(modify_poses_ui_->movement_type_modify_);
  checkboxes.push_back(modify_poses_ui_->polygon_end_modify_);
  checkboxes.push_back(modify_poses_ui_->polygon_start_modify_);
  checkboxes.push_back(modify_poses_ui_->pose_modify_);
  checkboxes.push_back(modify_poses_ui_->speed_modify_);

  std::vector<bool> checkbox_ticks;
  for (auto checkbox : checkboxes)
    checkbox_ticks.push_back(checkbox->isChecked());

  if (std::all_of(checkbox_ticks.begin(), checkbox_ticks.end(), [](bool v) {return !v;}))
  {
    Q_EMIT displayMessageBox("Modify",
                             "No item is ticked to be modified",
                             "Please tick at least one item",
                             QMessageBox::Icon::Critical);
    return;
  }

  // Some fields are left un-initialized, this is fine because
  // we are NOT using these fields. (eg: we cannot change the UUID of a pose!)
  ram_modify_trajectory::ModifySelectedPoses srv;
  srv.request.poses = selected_poses_;

  if (!modify_poses_ui_->pose_modify_->isChecked())
  {
    // Don't modify = send identity pose + relative mode
    Eigen::Affine3d id(Eigen::Affine3d::Identity());
    tf::poseEigenToMsg(id, srv.request.pose_reference.pose);
    srv.request.pose = false; // Relative mode
  }
  else
  {
    srv.request.pose = modify_poses_ui_->pose_abs_rel_->currentIndex();
    tf::poseEigenToMsg(modify_poses_ui_->pose_->pose(), srv.request.pose_reference.pose);
  }

  srv.request.pose_reference.params.approach_type = modify_poses_ui_->approach_type_->currentIndex();
  srv.request.approach_type = modify_poses_ui_->approach_type_modify_->isChecked();

  srv.request.movement_type = modify_poses_ui_->movement_type_modify_->isChecked();
  srv.request.pose_reference.params.movement_type = modify_poses_ui_->movement_type_->currentIndex();

  srv.request.polygon_end = modify_poses_ui_->polygon_end_modify_->isChecked();
  srv.request.pose_reference.polygon_end = modify_poses_ui_->polygon_end_->currentIndex();

  srv.request.polygon_start = modify_poses_ui_->polygon_start_modify_->isChecked();
  srv.request.pose_reference.polygon_start = modify_poses_ui_->polygon_start_->currentIndex();

  // Blend radius
  if (!modify_poses_ui_->blend_radius_modify_->isChecked())
  {
    // Don't modify = send 0 + relative mode
    srv.request.pose_reference.params.blend_radius = 0;
    srv.request.blend_radius = false; // Relative
  }
  else
  {
    srv.request.blend_radius = modify_poses_ui_->blend_radius_abs_rel_->currentIndex();
    srv.request.pose_reference.params.blend_radius = modify_poses_ui_->blend_radius_->value();
  }

  // Speed
  if (!modify_poses_ui_->speed_modify_->isChecked())
  {
    // Don't modify = send 0 + relative mode
    srv.request.pose_reference.params.speed = 0;
    srv.request.speed = false; // Relative
  }
  else
  {
    srv.request.speed = modify_poses_ui_->speed_abs_rel_->currentIndex();
    // Convert meters/min in meters/sec
    srv.request.pose_reference.params.speed = modify_poses_ui_->speed_->value()
        * modify_poses_ui_->speed_conversion_factor_;
  }

  // Laser power
  if (!modify_poses_ui_->laser_power_modify_->isChecked())
  {
    // Don't modify = send 0 + relative mode
    srv.request.pose_reference.params.laser_power = 0;
    srv.request.laser_power = false; // Relative
  }
  else
  {
    srv.request.laser_power = modify_poses_ui_->laser_power_abs_rel_->currentIndex();
    srv.request.pose_reference.params.laser_power = modify_poses_ui_->laser_power_->value();
  }

  // Feed rate
  if (!modify_poses_ui_->feed_rate_modify_->isChecked())
  {
    // Don't modify = send 0 + relative mode
    srv.request.pose_reference.params.feed_rate = 0;
    srv.request.feed_rate = false; // Relative
  }
  else
  {
    srv.request.feed_rate = modify_poses_ui_->feed_rate_abs_rel_->currentIndex();
    // Convert meters/min in meters/sec
    srv.request.pose_reference.params.feed_rate = modify_poses_ui_->feed_rate_->value() / 60.0;
  }

  if (!modify_selected_poses_client_.call(srv) || !srv.response.error.empty())
  {
    Q_EMIT displayMessageBox("Error",
                             "Could not modify the selection!",
                             "Make sure none of the values go below zero!" +
                             QString::fromStdString("\nError : " + srv.response.error),
                             QMessageBox::Icon::Critical);
  }

  changeGUIToSelectionMode();
}

void Modify::changeGUIToRotatePoses()
{
  Q_EMIT enable(false);
  clearLayout(layout_);

  rotation_angle_ = new QDoubleSpinBox;
  rotation_angle_->setRange(-180, 180);
  rotation_angle_->setSuffix(" °");

  QHBoxLayout *rotation_angle_layout = new QHBoxLayout;
  rotation_angle_layout->addWidget(new QLabel("Rotation angle in z-axis:"));
  rotation_angle_layout->addWidget(rotation_angle_);

  rotation_point_x_ = new QDoubleSpinBox;
  rotation_point_x_->setSuffix(" mm");
  rotation_point_x_->setRange(-9999, 9999);

  rotation_point_y_ = new QDoubleSpinBox;
  rotation_point_y_->setSuffix(" mm");
  rotation_point_y_->setRange(-9999, 9999);

  QHBoxLayout *rotation_point_layout = new QHBoxLayout;
  rotation_point_layout->addWidget(new QLabel("Rotation point:"));
  rotation_point_layout->addWidget(new QLabel("X:"));
  rotation_point_layout->addWidget(rotation_point_x_);
  rotation_point_layout->addWidget(new QLabel("Y:"));
  rotation_point_layout->addWidget(rotation_point_y_);

  layout_->addLayout(rotation_angle_layout);
  layout_->addLayout(rotation_point_layout);
  layout_->addStretch(1);

  QDialogButtonBox *button_box = new QDialogButtonBox(QDialogButtonBox::Ok
      | QDialogButtonBox::Cancel);
  layout_->addStretch(2);
  layout_->addWidget(button_box);

  connect(button_box, &QDialogButtonBox::accepted, this, &Modify::rotatePoses);
  connect(button_box, &QDialogButtonBox::rejected, this, &Modify::changeGUIToSelectionMode);

  Q_EMIT enable(true);
}

void Modify::rotatePoses()
{
  ram_modify_trajectory::RotateSelectedPoses srv;

  srv.request.poses = selected_poses_;

  srv.request.rotation_angle = rotation_angle_->value() * M_PI / 180.0;
  srv.request.center_of_rotation.x = rotation_point_x_->value() / 1000.0;
  srv.request.center_of_rotation.y = rotation_point_y_->value() / 1000.0;
  srv.request.center_of_rotation.z = 0;

  if (!rotate_selected_poses_client_.call(srv) || !srv.response.error.empty())
  {
    Q_EMIT displayMessageBox("Error",
                             "Could not rotate the selection!",
                             QString::fromStdString("Error : " + srv.response.error),
                             QMessageBox::Icon::Critical);
  }

  changeGUIToSelectionMode();
}

void Modify::changeGUIToReflectPoses()
{
  Q_EMIT enable(false);
  clearLayout(layout_);

  reflect_vector_x_ = new QDoubleSpinBox;
  reflect_vector_x_->setRange(-9999, 9999);
  reflect_vector_x_->setValue(1);

  reflect_vector_y_ = new QDoubleSpinBox;
  reflect_vector_y_->setRange(-9999, 9999);
  reflect_vector_y_->setValue(0);

  QHBoxLayout *reflect_vector_layout = new QHBoxLayout;
  reflect_vector_layout->addWidget(new QLabel("X:"));
  reflect_vector_layout->addWidget(reflect_vector_x_);
  reflect_vector_layout->addWidget(new QLabel("Y:"));
  reflect_vector_layout->addWidget(reflect_vector_y_);

  reflect_point_x_ = new QDoubleSpinBox;
  reflect_point_x_->setSuffix(" mm");
  reflect_point_x_->setRange(-9999, 9999);

  reflect_point_y_ = new QDoubleSpinBox;
  reflect_point_y_->setSuffix(" mm");
  reflect_point_y_->setRange(-9999, 9999);

  QHBoxLayout *reflect_point_layout = new QHBoxLayout;
  reflect_point_layout->addWidget(new QLabel("X:"));
  reflect_point_layout->addWidget(reflect_point_x_);
  reflect_point_layout->addWidget(new QLabel("Y:"));
  reflect_point_layout->addWidget(reflect_point_y_);

  layout_->addWidget(new QLabel("Normal vector to reflection plane:"));
  layout_->addLayout(reflect_vector_layout);
  layout_->addWidget(new QLabel("Point on reflection plane:"));
  layout_->addLayout(reflect_point_layout);
  layout_->addStretch(1);

  QDialogButtonBox *button_box = new QDialogButtonBox(QDialogButtonBox::Ok
      | QDialogButtonBox::Cancel);
  layout_->addStretch(2);
  layout_->addWidget(button_box);

  connect(button_box, &QDialogButtonBox::accepted, this, &Modify::reflectPoses);
  connect(button_box, &QDialogButtonBox::rejected, this, &Modify::changeGUIToSelectionMode);

  Q_EMIT enable(true);
}

void Modify::reflectPoses()
{
  ram_modify_trajectory::ReflectSelectedPoses srv;

  srv.request.poses = selected_poses_;

  srv.request.normal_vector.x = reflect_vector_x_->value() / 1000.0;
  srv.request.normal_vector.y = reflect_vector_y_->value() / 1000.0;
  srv.request.normal_vector.z = 0;

  srv.request.point_on_plane.x = reflect_point_x_->value() / 1000.0;
  srv.request.point_on_plane.y = reflect_point_y_->value() / 1000.0;
  srv.request.point_on_plane.z = 0;

  if (!reflect_selected_poses_client_.call(srv) || !srv.response.error.empty())
  {
    Q_EMIT displayMessageBox("Error",
                             "Could not reflect the selection!",
                             QString::fromStdString("Error : " + srv.response.error),
                             QMessageBox::Icon::Critical);
  }

  changeGUIToSelectionMode();

}

void Modify::changeGUIToScalePoses()
{
  Q_EMIT enable(false);
  clearLayout(layout_);

  scale_factor_ = new QDoubleSpinBox;
  scale_factor_->setRange(0, 999);
  scale_factor_->setValue(1);
  scale_factor_->setSingleStep(0.01);

  QHBoxLayout *scale_factor_layout = new QHBoxLayout;
  scale_factor_layout->addWidget(new QLabel("Scale factor :"));
  scale_factor_layout->addWidget(scale_factor_);

  scale_center_x_ = new QDoubleSpinBox;
  scale_center_x_->setSuffix(" mm");
  scale_center_x_->setRange(-9999, 9999);

  scale_center_y_ = new QDoubleSpinBox;
  scale_center_y_->setSuffix(" mm");
  scale_center_y_->setRange(-9999, 9999);

  QHBoxLayout *scale_center_layout = new QHBoxLayout;
  scale_center_layout->addWidget(new QLabel("X:"));
  scale_center_layout->addWidget(scale_center_x_);
  scale_center_layout->addStretch(1);
  scale_center_layout->addWidget(new QLabel("Y:"));
  scale_center_layout->addWidget(scale_center_y_);

  layout_->addLayout(scale_factor_layout);
  layout_->addWidget(new QLabel("Center of scaling :"));
  layout_->addLayout(scale_center_layout);
  layout_->addStretch(1);

  QDialogButtonBox *button_box = new QDialogButtonBox(QDialogButtonBox::Ok
      | QDialogButtonBox::Cancel);
  layout_->addStretch(2);
  layout_->addWidget(button_box);

  connect(button_box, &QDialogButtonBox::accepted, this, &Modify::scalePoses);
  connect(button_box, &QDialogButtonBox::rejected, this, &Modify::changeGUIToSelectionMode);

  Q_EMIT enable(true);
}

void Modify::scalePoses()
{
  Q_EMIT displayMessageBox("Warning",
                           "This operation modifies trajectory parameters",
                           "",
                           QMessageBox::Icon::Critical);

  ram_modify_trajectory::ScaleSelectedPoses srv;

  srv.request.poses = selected_poses_;

  srv.request.scale_factor = scale_factor_->value();

  srv.request.center_of_scaling.x = scale_center_x_->value() / 1000.0;
  srv.request.center_of_scaling.y = scale_center_x_->value() / 1000.0;
  srv.request.center_of_scaling.z = 0;

  if (!scale_selected_poses_client_.call(srv) || !srv.response.error.empty())
  {
    Q_EMIT displayMessageBox("Error",
                             "Could not reflect the selection!",
                             QString::fromStdString("Error : " + srv.response.error),
                             QMessageBox::Icon::Critical);
  }

  changeGUIToSelectionMode();
}

void Modify::changeGUIToShiftPoses()
{
  Q_EMIT enable(false);
  clearLayout(layout_);

  shift_direction_angle_ = new QDoubleSpinBox;
  shift_direction_angle_->setRange(-180, 180);
  shift_direction_angle_->setSingleStep(10);
  shift_direction_angle_->setSuffix(" deg");

  shift_angle_z_ = new QDoubleSpinBox;
  shift_angle_z_->setRange(-90, 90);
  shift_angle_z_->setSingleStep(1);
  shift_angle_z_->setSuffix(" deg");

  QVBoxLayout *form = new QVBoxLayout;
  QLabel *shift_direction_angle_label = new QLabel("Shift direction angle: ");
  shift_direction_angle_label->setToolTip("The direction of the shifting, 0 deg means X axis, 90 deg means Y axis");
  QHBoxLayout *shift_direction_angle_layout = new QHBoxLayout;
  shift_direction_angle_layout->addWidget(shift_direction_angle_label);
  shift_direction_angle_layout->addWidget(shift_direction_angle_);
  form->addLayout(shift_direction_angle_layout);

  QLabel *shift_angle_z_label = new QLabel("Z shift angle: ");
  shift_angle_z_label->setToolTip("The angle of shifting along the Z axis");
  QHBoxLayout *shift_angle_z_layout = new QHBoxLayout;
  shift_angle_z_layout->addWidget(shift_angle_z_label);
  shift_angle_z_layout->addWidget(shift_angle_z_);
  form->addLayout(shift_angle_z_layout);

  layout_->addLayout(form);
  layout_->addStretch(1);

  QDialogButtonBox *button_box = new QDialogButtonBox(QDialogButtonBox::Ok
      | QDialogButtonBox::Cancel);
  layout_->addStretch(2);
  layout_->addWidget(button_box);

  connect(button_box, &QDialogButtonBox::accepted, this, &Modify::shiftPoses);
  connect(button_box, &QDialogButtonBox::rejected, this, &Modify::changeGUIToSelectionMode);

  Q_EMIT enable(true);
}

void Modify::shiftPoses()
{
  ram_modify_trajectory::ShiftPoses srv;
  srv.request.poses = selected_poses_;
  srv.request.angle_z = shift_angle_z_->value() * M_PI / 180.0;
  srv.request.direction_angle = shift_direction_angle_->value() * M_PI / 180.0;

  if (!shift_poses_client_.call(srv) || !srv.response.error.empty())
  {
    Q_EMIT displayMessageBox("Error",
                             "Could not shift the selection!",
                             QString::fromStdString("Error : " + srv.response.error),
                             QMessageBox::Icon::Critical);
  }

  changeGUIToSelectionMode();
}

void Modify::connectToService(ros::ServiceClient &client)
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

void Modify::connectToServices()
{
  Q_EMIT enable(false);

  // Display
  connectToService(update_selection_client_);

  // Pose selector
  connectToService(get_trajectory_size_client_);
  connectToService(get_number_of_layers_client_);
  connectToService(get_layer_size_client_);
  connectToService(get_poses_from_trajectory_client_);
  connectToService(get_poses_from_layers_list_client_);
  connectToService(get_poses_from_layer_client_);
  connectToService(get_poses_from_information_);

  // Modify services
  connectToService(modify_selected_poses_client_);
  connectToService(add_poses_client_);
  connectToService(reset_selected_poses_client_);
  connectToService(delete_selected_poses_client_);
  connectToService(trajectory_interruption_client_);
  connectToService(change_layer_height_client_);
  connectToService(tweak_blend_radius_client_);
  connectToService(simplify_trajectory_client_);

  // Geometric operation services
  connectToService(rotate_selected_poses_client_);
  connectToService(reflect_selected_poses_client_);

  ROS_INFO_STREAM("RViz panel " << getName().toStdString() << " services connections have been made");

  Q_EMIT enable(true);
}

void Modify::load(const rviz::Config &config)
{
  int tmp;
  Q_EMIT configChanged();
  if (config.mapGetInt("selection_mode", &tmp))
  {
    selection_mode_ = (unsigned) tmp;
    selection_buttons_.at(selection_mode_)->setChecked(true);
  }
  rviz::Panel::load(config);
}

void Modify::save(rviz::Config config) const
{
  config.mapSetValue("selection_mode", selection_mode_);
  rviz::Panel::save(config);
}

void Modify::displayMessageBoxHandler(const QString title,
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

void Modify::updateTemporarySelection(std::vector<unsigned> selection)
{
  ram_display::UpdateSelection srv_tmp_select;
  srv_tmp_select.request.temporary = true;

  switch (selection_mode_)
  {
    case 0:
      {
        // Nothing to do here
      }
    case 1:
      {
        ram_modify_trajectory::GetPosesFromTrajectory srv;
        srv.request.pose_index_list = selection;
        get_poses_from_trajectory_client_.call(srv);
        srv_tmp_select.request.selected_poses = srv.response.poses;
        break;
      }
    case 2:
      {
        ram_modify_trajectory::GetPosesFromLayersList srv;
        srv.request.layer_level_list = selection;
        get_poses_from_layers_list_client_.call(srv);
        srv_tmp_select.request.selected_poses = srv.response.poses;
        break;
      }
    case 3:
      {
        if (!is_propagating_)
        {
          ram_modify_trajectory::GetPosesFromLayer srv;
          srv.request.index_list_relative = selection;
          srv.request.layer_level = layer_level_;

          if (!get_poses_from_layer_client_.call(srv))
            srv_tmp_select.request.selected_poses.clear();
          else
            srv_tmp_select.request.selected_poses = srv.response.poses;
        }
        else
        {
          // Call GetPosesFromLayer multiple times and check whether this is ok
          for (auto &selected : selection)
          {
            ram_modify_trajectory::GetPosesFromLayer srv;
            srv.request.index_list_relative = relative_indices_;
            srv.request.layer_level = selected;
            if (!get_poses_from_layer_client_.call(srv)) // If one of the call fails, don't display anything
            {
              srv_tmp_select.request.selected_poses.clear();
              break;
            }
            else
            {
              // Append to selection vector
              srv_tmp_select.request.selected_poses.insert(srv_tmp_select.request.selected_poses.end(),
                  srv.response.poses.begin(),
                  srv.response.poses.end());
            }
          }
        }
        break;
      }
    default:
      {
        ROS_ERROR_STREAM("Missing implementation in Modify Qt panel inside updateTemporarySelection");
        return;
      }
  }

  if (!update_selection_client_.call(srv_tmp_select))
    ROS_WARN_STREAM("Could not update temporary selection in Modify Qt panel");
}

}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(ram_qt_guis::Modify, rviz::Panel)
