#include <ram_qt_guis/path_planning.hpp>

namespace ram_qt_guis
{

PathPlanning::PathPlanning(QWidget *parent) :
  rviz::Panel(parent)
{
  connect(this, &PathPlanning::enable, this, &PathPlanning::setEnabled);
  setObjectName("PathPlanning");
  setName(objectName());

  QHBoxLayout *select_algorithm_layout = new QHBoxLayout;
  select_algorithm_layout->addWidget(new QLabel("Generation algorithm:"));

  select_algorithm_ = new QComboBox;
  select_algorithm_layout->addWidget(select_algorithm_);

  algorithm_description_ = new QLabel;
  algorithm_description_->setWordWrap(true);
  algorithm_stacked_widget_ = new QStackedWidget;
  generate_trajectory_button_ = new QPushButton("Generate trajectory");

  // Scroll area
  QVBoxLayout *scroll_widget_layout = new QVBoxLayout();
  QWidget *scroll_widget = new QWidget;
  scroll_widget->setLayout(scroll_widget_layout);
  QScrollArea *scroll_area = new QScrollArea;
  scroll_area->setWidget(scroll_widget);
  scroll_area->setWidgetResizable(true);
  scroll_area->setFrameShape(QFrame::NoFrame);

  // Main layout
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  main_layout->addWidget(scroll_area);
  scroll_widget_layout->addLayout(select_algorithm_layout);
  scroll_widget_layout->addWidget(algorithm_description_);
  scroll_widget_layout->addWidget(algorithm_stacked_widget_);
  scroll_widget_layout->addWidget(generate_trajectory_button_);
  scroll_widget_layout->addStretch(1);

  qRegisterMetaType<QMessageBox::Icon>();
  connect(this, &PathPlanning::displayMessageBox, this, &PathPlanning::displayMessageBoxHandler);

  // Algorithm generators
  ram_path_planning::DonghongDing<ram_path_planning::DonghongDingAction> donghong_ding_generator;
  ram_path_planning::Contours<ram_path_planning::ContoursAction> contours_generator;
  ram_path_planning::FollowPoses<ram_path_planning::FollowPosesAction> follow_poses_generator;
  ram_path_planning::Revolve<ram_path_planning::RevolveAction> revolve_generator;
  ram_path_planning::PolygonOffsets<ram_path_planning::PolygonOffsetsAction> polygon_offsets_generator;
  ram_path_planning::Profile<ram_path_planning::ProfileAction> profile_generator;

  // Add algorithms names
  select_algorithm_->addItem(QString::fromStdString(donghong_ding_generator.name_));
  select_algorithm_->addItem(QString::fromStdString(contours_generator.name_));
  select_algorithm_->addItem(QString::fromStdString(follow_poses_generator.name_));
  select_algorithm_->addItem(QString::fromStdString(revolve_generator.name_));
  select_algorithm_->addItem(QString::fromStdString(polygon_offsets_generator.name_));
  select_algorithm_->addItem(QString::fromStdString(profile_generator.name_));

  // Add algorithms descriptions
  algorithm_descriptions_.clear();
  algorithm_descriptions_.push_back(donghong_ding_generator.description_);
  algorithm_descriptions_.push_back(contours_generator.description_);
  algorithm_descriptions_.push_back(follow_poses_generator.description_);
  algorithm_descriptions_.push_back(revolve_generator.description_);
  algorithm_descriptions_.push_back(polygon_offsets_generator.description_);
  algorithm_descriptions_.push_back(profile_generator.description_);

  // Algorithm widget
  donghong_ding_ui_ = new DonghongDingWidget();
  contours_ui_ = new ContoursWidget();
  follow_poses_ui_ = new FollowPosesWidget();
  revolve_ui_ = new RevolveWidget();
  polygon_offsets_ui_ = new PolygonOffsetsWidget();
  profile_ui_ = new ProfileWidget();

  // Add algorithm widget
  algorithm_stacked_widget_->addWidget(donghong_ding_ui_);
  algorithm_stacked_widget_->addWidget(contours_ui_);
  algorithm_stacked_widget_->addWidget(follow_poses_ui_);
  algorithm_stacked_widget_->addWidget(revolve_ui_);
  algorithm_stacked_widget_->addWidget(polygon_offsets_ui_);
  algorithm_stacked_widget_->addWidget(profile_ui_);

  // Action clients
  donghong_ding_ac_.reset(new DonghongDingActionClient(donghong_ding_generator.service_name_, true));
  contours_ac_.reset(new ContoursActionClient(contours_generator.service_name_, true));
  follow_poses_ac_.reset(new FollowPosesActionClient(follow_poses_generator.service_name_, true));
  revolve_ac_.reset(new RevolveActionClient(revolve_generator.service_name_, true));
  polygon_offsets_ac_.reset(new PolygonOffsetsActionClient(polygon_offsets_generator.service_name_, true));
  profile_ac_.reset(new ProfileActionClient(profile_generator.service_name_, true));

  connect(donghong_ding_ui_, &DonghongDingWidget::valueChanged, this, &PathPlanning::configChanged);
  connect(contours_ui_, &ContoursWidget::valueChanged, this, &PathPlanning::configChanged);
  connect(follow_poses_ui_, &FollowPosesWidget::valueChanged, this, &PathPlanning::configChanged);
  connect(revolve_ui_, &RevolveWidget::valueChanged, this, &PathPlanning::configChanged);
  connect(polygon_offsets_ui_, &PolygonOffsetsWidget::valueChanged, this, &PathPlanning::configChanged);
  connect(profile_ui_, &ProfileWidget::valueChanged, this, &PathPlanning::configChanged);

  progress_dialog_ = new QProgressDialog;
  connect(this, &PathPlanning::updateProgressDialogValue, progress_dialog_, &QProgressDialog::setValue);
  connect(this, &PathPlanning::updateProgressDialogText, progress_dialog_, &QProgressDialog::setLabelText);
  progress_dialog_->setModal(true);
  progress_dialog_->reset(); // Otherwise it will show up after minimumDuration

  connect(algorithm_stacked_widget_, qOverload<int>(&QStackedWidget::currentChanged), this, [=]()
  {
    for (int i(0); i < algorithm_stacked_widget_->count(); ++i)
    {
      QSizePolicy::Policy policy = QSizePolicy::Ignored;
      if (i == algorithm_stacked_widget_->currentIndex())
        policy = QSizePolicy::Expanding;
      QWidget *page = algorithm_stacked_widget_->widget(i);
      page->setSizePolicy(policy, policy);
    }
  });
  Q_EMIT algorithm_stacked_widget_->currentChanged(0);
}

PathPlanning::~PathPlanning()
{
}

void PathPlanning::connectToActions()
{
  Q_EMIT enable(false);

  // Generate trajectory
  donghong_ding_ac_->waitForServer();
  contours_ac_->waitForServer();
  follow_poses_ac_->waitForServer();
  revolve_ac_->waitForServer();
  polygon_offsets_ac_->waitForServer();
  profile_ac_->waitForServer();

  ROS_INFO_STREAM("RViz panel " << getName().toStdString() << " actions connections have been made");

  connect(select_algorithm_, qOverload<int>(&QComboBox::currentIndexChanged), this, &PathPlanning::algorithmChanged);
  connect(select_algorithm_, qOverload<int>(&QComboBox::currentIndexChanged), this, &PathPlanning::configChanged);

  select_algorithm_->setCurrentIndex(loaded_last_algorithm_);
  Q_EMIT select_algorithm_->currentIndexChanged(loaded_last_algorithm_);
  Q_EMIT enable(true);
}

void PathPlanning::algorithmChanged()
{
  Q_EMIT enable(false);

  generate_trajectory_button_->disconnect();

  if ((unsigned) select_algorithm_->currentIndex() >= algorithm_descriptions_.size())
    select_algorithm_->setCurrentIndex(0);

  // Change description
  algorithm_description_->setText(QString::fromStdString(algorithm_descriptions_.at(select_algorithm_->currentIndex())));
  // Change widget
  algorithm_stacked_widget_->setCurrentIndex(select_algorithm_->currentIndex());

  // Connect button
  disconnect(progress_dialog_, &QProgressDialog::canceled, 0, 0);
  switch (select_algorithm_->currentIndex())
  {
    case 0:
    {
      connect(generate_trajectory_button_, &QPushButton::clicked, this, &PathPlanning::donghongDingButtonHandler);
      connect(progress_dialog_, &QProgressDialog::canceled, this, &PathPlanning::donghongDingCancel);
      break;
    }
    case 1:
    {
      connect(generate_trajectory_button_, &QPushButton::clicked, this, &PathPlanning::contoursButtonHandler);
      connect(progress_dialog_, &QProgressDialog::canceled, this, &PathPlanning::contoursCancel);
      break;
    }
    case 2:
    {
      connect(generate_trajectory_button_, &QPushButton::clicked, this, &PathPlanning::followPosesButtonHandler);
      connect(progress_dialog_, &QProgressDialog::canceled, this, &PathPlanning::followPosesCancel);
      break;
    }
    case 3:
    {
      connect(generate_trajectory_button_, &QPushButton::clicked, this, &PathPlanning::revolveButtonHandler);
      connect(progress_dialog_, &QProgressDialog::canceled, this, &PathPlanning::revolveCancel);
      break;
    }
    case 4:
    {
      connect(generate_trajectory_button_, &QPushButton::clicked, this, &PathPlanning::polygonOffsetsButtonHandler);
      connect(progress_dialog_, &QProgressDialog::canceled, this, &PathPlanning::polygonOffsetsCancel);
      break;
    }
    case 5:
    {
      connect(generate_trajectory_button_, &QPushButton::clicked, this, &PathPlanning::profileButtonHandler);
      connect(progress_dialog_, &QProgressDialog::canceled, this, &PathPlanning::profileCancel);
      break;
    }
    default:
      Q_EMIT displayMessageBox("Operation", "Error selecting algorithm", "", QMessageBox::Icon::Critical);
      break;
  }

  Q_EMIT enable(true);
}

// DonghongDing
void PathPlanning::donghongDingButtonHandler()
{
  Q_EMIT configChanged();
  Q_EMIT enable(false);

  std::string error = donghong_ding_ui_->fillGoal(donghong_ding_goal_);
  if (!error.empty())
  {
    Q_EMIT displayMessageBox("Error filling goal", QString::fromStdString(error), "", QMessageBox::Icon::Critical);
    Q_EMIT enable(true);
    return;
  }

  donghong_ding_ac_->sendGoal(donghong_ding_goal_, boost::bind(&PathPlanning::donghongDingDoneCb, this, _1, _2), NULL,
                              boost::bind(&PathPlanning::donghongDingFeedbackCb, this, _1));
}

void PathPlanning::donghongDingCancel()
{
  Q_EMIT enable(true);
  if (donghong_ding_ac_->getState().state_ == actionlib::SimpleClientGoalState::ACTIVE)
    donghong_ding_ac_->cancelAllGoals();

  progress_dialog_->reset();
}

void PathPlanning::donghongDingDoneCb(const actionlib::SimpleClientGoalState &state,
                                      const ram_path_planning::DonghongDingResultConstPtr &result)
{
  Q_EMIT enable(true);

  if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED ||
      state.state_ == actionlib::SimpleClientGoalState::ABORTED)
  {
    if (!result->error_msg.empty())
    {
      Q_EMIT progress_dialog_->canceled();
      Q_EMIT displayMessageBox("Path planning failed",
                               QString::fromStdString(result->error_msg),
                               "", QMessageBox::Icon::Critical);
      return;
    }
  }
  else if (state.state_ == actionlib::SimpleClientGoalState::PREEMPTED)
  {
    Q_EMIT progress_dialog_->canceled();
    return;
  }

  Q_EMIT progress_dialog_->accepted();
}

void PathPlanning::donghongDingFeedbackCb(const ram_path_planning::DonghongDingFeedbackConstPtr &feedback)
{
  if (progress_dialog_->wasCanceled())
    return;

  Q_EMIT updateProgressDialogText(QString::fromStdString(feedback->progress_msg));
  Q_EMIT updateProgressDialogValue(feedback->progress_value);
}

// Contours
void PathPlanning::contoursButtonHandler()
{
  Q_EMIT configChanged();
  Q_EMIT enable(false);

  std::string error = contours_ui_->fillGoal(contours_goal_);
  if (!error.empty())
  {
    Q_EMIT displayMessageBox("Error filling goal", QString::fromStdString(error), "", QMessageBox::Icon::Critical);
    Q_EMIT enable(true);
    return;
  }

  contours_ac_->sendGoal(contours_goal_, boost::bind(&PathPlanning::contoursDoneCb, this, _1, _2), NULL,
                         boost::bind(&PathPlanning::contoursFeedbackCb, this, _1));
}

void PathPlanning::contoursCancel()
{
  Q_EMIT enable(true);
  if (contours_ac_->getState().state_ == actionlib::SimpleClientGoalState::ACTIVE)
    contours_ac_->cancelAllGoals();

  progress_dialog_->reset();
}

void PathPlanning::contoursDoneCb(const actionlib::SimpleClientGoalState &state,
                                  const ram_path_planning::ContoursResultConstPtr &result)
{
  Q_EMIT enable(true);

  if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED ||
      state.state_ == actionlib::SimpleClientGoalState::ABORTED)
  {
    if (!result->error_msg.empty())
    {
      Q_EMIT progress_dialog_->canceled();
      Q_EMIT displayMessageBox("Path planning failed",
                               QString::fromStdString(result->error_msg),
                               "", QMessageBox::Icon::Critical);
      return;
    }
  }
  else if (state.state_ == actionlib::SimpleClientGoalState::PREEMPTED)
  {
    Q_EMIT progress_dialog_->canceled();
    return;
  }

  Q_EMIT progress_dialog_->accepted();
}

void PathPlanning::contoursFeedbackCb(const ram_path_planning::ContoursFeedbackConstPtr &feedback)
{
  if (progress_dialog_->wasCanceled())
    return;

  Q_EMIT updateProgressDialogText(QString::fromStdString(feedback->progress_msg));
  Q_EMIT updateProgressDialogValue(feedback->progress_value);
}

// Follow poses
void PathPlanning::followPosesButtonHandler()
{
  Q_EMIT configChanged();
  Q_EMIT enable(false);

  std::string error = follow_poses_ui_->fillGoal(follow_poses_goal_);
  if (!error.empty())
  {
    Q_EMIT displayMessageBox("Error filling goal", QString::fromStdString(error), "", QMessageBox::Icon::Critical);
    Q_EMIT enable(true);
    return;
  }

  follow_poses_ac_->sendGoal(follow_poses_goal_, boost::bind(&PathPlanning::followPosesDoneCb, this, _1, _2), NULL,
                             boost::bind(&PathPlanning::followPosesFeedbackCb, this, _1));
}

void PathPlanning::followPosesCancel()
{
  Q_EMIT enable(true);
  if (follow_poses_ac_->getState().state_ == actionlib::SimpleClientGoalState::ACTIVE)
    follow_poses_ac_->cancelAllGoals();

  progress_dialog_->reset();
}

void PathPlanning::followPosesDoneCb(const actionlib::SimpleClientGoalState &state,
                                     const ram_path_planning::FollowPosesResultConstPtr &result)
{
  Q_EMIT enable(true);

  if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED ||
      state.state_ == actionlib::SimpleClientGoalState::ABORTED)
  {
    if (!result->error_msg.empty())
    {
      Q_EMIT progress_dialog_->canceled();
      Q_EMIT displayMessageBox("Path planning failed",
                               QString::fromStdString(result->error_msg),
                               "", QMessageBox::Icon::Critical);
      return;
    }
  }
  else if (state.state_ == actionlib::SimpleClientGoalState::PREEMPTED)
  {
    Q_EMIT progress_dialog_->canceled();
    return;
  }

  Q_EMIT progress_dialog_->accepted();
}

void PathPlanning::followPosesFeedbackCb(const ram_path_planning::FollowPosesFeedbackConstPtr &feedback)
{
  if (progress_dialog_->wasCanceled())
    return;

  Q_EMIT updateProgressDialogText(QString::fromStdString(feedback->progress_msg));
  Q_EMIT updateProgressDialogValue(feedback->progress_value);
}

// Revolve
void PathPlanning::revolveButtonHandler()
{
  Q_EMIT configChanged();
  Q_EMIT enable(false);

  std::string error = revolve_ui_->fillGoal(revolve_goal_);
  if (!error.empty())
  {
    Q_EMIT displayMessageBox("Error filling goal", QString::fromStdString(error), "", QMessageBox::Icon::Critical);
    Q_EMIT enable(true);
    return;
  }

  revolve_ac_->sendGoal(revolve_goal_, boost::bind(&PathPlanning::revolveDoneCb, this, _1, _2), NULL,
                        boost::bind(&PathPlanning::revolveFeedbackCb, this, _1));
}

void PathPlanning::revolveCancel()
{
  Q_EMIT enable(true);
  if (revolve_ac_->getState().state_ == actionlib::SimpleClientGoalState::ACTIVE)
    revolve_ac_->cancelAllGoals();

  progress_dialog_->reset();
}

void PathPlanning::revolveDoneCb(const actionlib::SimpleClientGoalState &state,
                                 const ram_path_planning::RevolveResultConstPtr &result)
{
  Q_EMIT enable(true);

  if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED ||
      state.state_ == actionlib::SimpleClientGoalState::ABORTED)
  {
    if (!result->error_msg.empty())
    {
      Q_EMIT progress_dialog_->canceled();
      Q_EMIT displayMessageBox("Path planning failed",
                               QString::fromStdString(result->error_msg),
                               "", QMessageBox::Icon::Critical);
      return;
    }
  }
  else if (state.state_ == actionlib::SimpleClientGoalState::PREEMPTED)
  {
    Q_EMIT progress_dialog_->canceled();
    return;
  }

  Q_EMIT progress_dialog_->accepted();
}

void PathPlanning::revolveFeedbackCb(const ram_path_planning::RevolveFeedbackConstPtr &feedback)
{
  if (progress_dialog_->wasCanceled())
    return;

  Q_EMIT updateProgressDialogText(QString::fromStdString(feedback->progress_msg));
  Q_EMIT updateProgressDialogValue(feedback->progress_value);
}

// Polygon offsets
void PathPlanning::polygonOffsetsButtonHandler()
{
  Q_EMIT configChanged();
  Q_EMIT enable(false);

  std::string error = polygon_offsets_ui_->fillGoal(polygon_offsets_goal_);
  if (!error.empty())
  {
    Q_EMIT displayMessageBox("Error filling goal", QString::fromStdString(error), "", QMessageBox::Icon::Critical);
    Q_EMIT enable(true);
    return;
  }

  polygon_offsets_ac_->sendGoal(polygon_offsets_goal_, boost::bind(&PathPlanning::polygonOffsetsDoneCb, this, _1, _2), NULL,
                        boost::bind(&PathPlanning::polygonOffsetsFeedbackCb, this, _1));
}

void PathPlanning::polygonOffsetsCancel()
{
  Q_EMIT enable(true);
  if (polygon_offsets_ac_->getState().state_ == actionlib::SimpleClientGoalState::ACTIVE)
    polygon_offsets_ac_->cancelAllGoals();

  progress_dialog_->reset();
}

void PathPlanning::polygonOffsetsDoneCb(const actionlib::SimpleClientGoalState &state,
                                        const ram_path_planning::PolygonOffsetsResultConstPtr &result)
{
    Q_EMIT enable(true);

  if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED ||
      state.state_ == actionlib::SimpleClientGoalState::ABORTED)
  {
    if (!result->error_msg.empty())
    {
      Q_EMIT progress_dialog_->canceled();
      Q_EMIT displayMessageBox("Path planning failed",
                               QString::fromStdString(result->error_msg),
                               "", QMessageBox::Icon::Critical);
      return;
    }
  }
  else if (state.state_ == actionlib::SimpleClientGoalState::PREEMPTED)
  {
    Q_EMIT progress_dialog_->canceled();
    return;
  }

  Q_EMIT progress_dialog_->accepted();
}

void PathPlanning::polygonOffsetsFeedbackCb(const ram_path_planning::PolygonOffsetsFeedbackConstPtr &feedback)
{
  if (progress_dialog_->wasCanceled())
    return;

  Q_EMIT updateProgressDialogText(QString::fromStdString(feedback->progress_msg));
  Q_EMIT updateProgressDialogValue(feedback->progress_value);
}

// Profile
void PathPlanning::profileButtonHandler()
{
  Q_EMIT configChanged();
  Q_EMIT enable(false);

  std::string error = profile_ui_->fillGoal(profile_goal_);
  if (!error.empty())
  {
    Q_EMIT displayMessageBox("Error filling goal", QString::fromStdString(error), "", QMessageBox::Icon::Critical);
    Q_EMIT enable(true);
    return;
  }

  profile_ac_->sendGoal(profile_goal_, boost::bind(&PathPlanning::profileDoneCb, this, _1, _2), NULL,
                        boost::bind(&PathPlanning::profileFeedbackCb, this, _1));
}

void PathPlanning::profileCancel()
{
  Q_EMIT enable(true);
  if (profile_ac_->getState().state_ == actionlib::SimpleClientGoalState::ACTIVE)
    profile_ac_->cancelAllGoals();

  progress_dialog_->reset();
}

void PathPlanning::profileDoneCb(const actionlib::SimpleClientGoalState &state,
                                 const ram_path_planning::ProfileResultConstPtr &result)
{
  Q_EMIT enable(true);

  if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED ||
      state.state_ == actionlib::SimpleClientGoalState::ABORTED)
  {
    if (!result->error_msg.empty())
    {
      Q_EMIT progress_dialog_->canceled();
      Q_EMIT displayMessageBox("Path planning failed",
                               QString::fromStdString(result->error_msg),
                               "", QMessageBox::Icon::Critical);
      return;
    }
  }
  else if (state.state_ == actionlib::SimpleClientGoalState::PREEMPTED)
  {
    Q_EMIT progress_dialog_->canceled();
    return;
  }

  Q_EMIT progress_dialog_->accepted();
}

void PathPlanning::profileFeedbackCb(const ram_path_planning::ProfileFeedbackConstPtr &feedback)
{
  if (progress_dialog_->wasCanceled())
    return;

  Q_EMIT updateProgressDialogText(QString::fromStdString(feedback->progress_msg));
  Q_EMIT updateProgressDialogValue(feedback->progress_value);
}

void PathPlanning::displayMessageBoxHandler(const QString title,
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

void PathPlanning::load(const rviz::Config &config)
{
  Q_EMIT configChanged();

  int tmp_int(0);
  if (config.mapGetInt("algorithm", &tmp_int))
    loaded_last_algorithm_ = tmp_int;

  donghong_ding_ui_->load(config);
  contours_ui_->load(config);
  follow_poses_ui_->load(config);
  revolve_ui_->load(config);
  polygon_offsets_ui_->load(config);
  profile_ui_->load(config);
  rviz::Panel::load(config);

  // Check connection of client
  QtConcurrent::run(this, &PathPlanning::connectToActions);
}

void PathPlanning::save(rviz::Config config) const
{
  config.mapSetValue("algorithm", select_algorithm_->currentIndex());

  donghong_ding_ui_->save(config);
  contours_ui_->save(config);
  follow_poses_ui_->save(config);
  revolve_ui_->save(config);
  polygon_offsets_ui_->save(config);
  profile_ui_->save(config);
  rviz::Panel::save(config);
}

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ram_qt_guis::PathPlanning, rviz::Panel)
