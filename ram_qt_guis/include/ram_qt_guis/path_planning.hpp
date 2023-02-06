#ifndef RAM_QT_GUIS_PATH_PLANNING_HPP
#define RAM_QT_GUIS_PATH_PLANNING_HPP

#ifndef Q_MOC_RUN
#include <actionlib/client/simple_action_client.h>
#include <QComboBox>
#include <QLabel>
#include <QMessageBox>
#include <QProgressDialog>
#include <QPushButton>
#include <QStackedWidget>
#include <QtConcurrent/QtConcurrent>
#include <ram_path_planning/ContoursAction.h>
#include <ram_path_planning/contours.hpp>
#include <ram_path_planning/DonghongDingAction.h>
#include <ram_path_planning/donghong_ding.hpp>
#include <ram_path_planning/FollowPosesAction.h>
#include <ram_path_planning/follow_poses.hpp>
#include <ram_path_planning/PolygonOffsetsAction.h>
#include <ram_path_planning/polygon_offsets.hpp>
#include <ram_path_planning/ProfileAction.h>
#include <ram_path_planning/profile.hpp>
#include <ram_path_planning/RevolveAction.h>
#include <ram_path_planning/revolve.hpp>
#include <ram_qt_guis/path_planning_widgets/contours.hpp>
#include <ram_qt_guis/path_planning_widgets/donghong_ding.hpp>
#include <ram_qt_guis/path_planning_widgets/follow_poses.hpp>
#include <ram_qt_guis/path_planning_widgets/polygon_offsets.hpp>
#include <ram_qt_guis/path_planning_widgets/profile.hpp>
#include <ram_qt_guis/path_planning_widgets/revolve.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>
#endif

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QStackedWidget>
#include <QtConcurrent/QtConcurrentRun>

namespace ram_qt_guis
{
class PathPlanning : public rviz::Panel
{
Q_OBJECT

public:
  PathPlanning(QWidget* parent = NULL);
  virtual ~PathPlanning();

Q_SIGNALS:
  void enable(const bool);
  void updateProgressDialogValue(const unsigned progress);
  void updateProgressDialogText(const QString message);
  void displayMessageBox(const QString,
                         const QString,
                         const QString,
                         const QMessageBox::Icon);

protected Q_SLOTS:
  void algorithmChanged();

  void donghongDingButtonHandler();
  void donghongDingCancel();
  void donghongDingDoneCb(const actionlib::SimpleClientGoalState &state,
              const ram_path_planning::DonghongDingResultConstPtr &result);
  void donghongDingFeedbackCb(const ram_path_planning::DonghongDingFeedbackConstPtr &feedback);

  void contoursButtonHandler();
  void contoursCancel();
  void contoursDoneCb(const actionlib::SimpleClientGoalState &state,
                      const ram_path_planning::ContoursResultConstPtr &result);
  void contoursFeedbackCb(const ram_path_planning::ContoursFeedbackConstPtr &feedback);

  void followPosesButtonHandler();
  void followPosesCancel();
  void followPosesDoneCb(const actionlib::SimpleClientGoalState& state,
                         const ram_path_planning::FollowPosesResultConstPtr& result);
  void followPosesFeedbackCb(const ram_path_planning::FollowPosesFeedbackConstPtr& feedback);

  void revolveButtonHandler();
  void revolveCancel();
  void revolveDoneCb(const actionlib::SimpleClientGoalState& state,
                     const ram_path_planning::RevolveResultConstPtr& result);
  void revolveFeedbackCb(const ram_path_planning::RevolveFeedbackConstPtr& feedback);

  void polygonOffsetsButtonHandler();
  void polygonOffsetsCancel();
  void polygonOffsetsDoneCb(const actionlib::SimpleClientGoalState& state,
                     const ram_path_planning::PolygonOffsetsResultConstPtr& result);
  void polygonOffsetsFeedbackCb(const ram_path_planning::PolygonOffsetsFeedbackConstPtr& feedback);

  void profileButtonHandler();
  void profileCancel();
  void profileDoneCb(const actionlib::SimpleClientGoalState& state,
                     const ram_path_planning::ProfileResultConstPtr& result);
  void profileFeedbackCb(const ram_path_planning::ProfileFeedbackConstPtr& feedback);

  void displayMessageBoxHandler(const QString title,
                                const QString text,
                                const QString info = "",
                                const QMessageBox::Icon icon = QMessageBox::Icon::Information);

  void load(const rviz::Config& config);
  void save(rviz::Config config) const;

private:
  void connectToActions();
  ros::NodeHandle nh_;

  QProgressDialog *progress_dialog_;
  QStackedWidget * algorithm_stacked_widget_;
  QPushButton *generate_trajectory_button_;

  QComboBox *select_algorithm_;
  QLabel *algorithm_description_;
  std::vector<std::string> algorithm_descriptions_;
  unsigned loaded_last_algorithm_ = 0;

  DonghongDingWidget *donghong_ding_ui_;
  ContoursWidget *contours_ui_;
  FollowPosesWidget *follow_poses_ui_;
  RevolveWidget *revolve_ui_;
  PolygonOffsetsWidget *polygon_offsets_ui_;
  ProfileWidget *profile_ui_;

  // Action clients
  using DonghongDingActionClient = actionlib::SimpleActionClient<ram_path_planning::DonghongDingAction>;
  using ContoursActionClient = actionlib::SimpleActionClient<ram_path_planning::ContoursAction>;
  using FollowPosesActionClient = actionlib::SimpleActionClient<ram_path_planning::FollowPosesAction>;
  using RevolveActionClient = actionlib::SimpleActionClient<ram_path_planning::RevolveAction>;
  using ProfileActionClient = actionlib::SimpleActionClient<ram_path_planning::ProfileAction>;
  using PolygonOffsetsActionClient = actionlib::SimpleActionClient<ram_path_planning::PolygonOffsetsAction>;

  std::unique_ptr<DonghongDingActionClient> donghong_ding_ac_;
  std::unique_ptr<ContoursActionClient> contours_ac_;
  std::unique_ptr<FollowPosesActionClient> follow_poses_ac_;
  std::unique_ptr<RevolveActionClient> revolve_ac_;
  std::unique_ptr<PolygonOffsetsActionClient> polygon_offsets_ac_;
  std::unique_ptr<ProfileActionClient> profile_ac_;

  // Goals
  ram_path_planning::DonghongDingGoal donghong_ding_goal_;
  ram_path_planning::ContoursGoal contours_goal_;
  ram_path_planning::FollowPosesGoal follow_poses_goal_;
  ram_path_planning::RevolveGoal revolve_goal_;
  ram_path_planning::PolygonOffsetsGoal polygon_offsets_goal_;
  ram_path_planning::ProfileGoal profile_goal_;
};

}

#endif
