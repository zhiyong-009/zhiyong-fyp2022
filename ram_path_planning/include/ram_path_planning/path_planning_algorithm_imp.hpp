#ifndef RAM_PATH_PLANNING_PATH_PLANNING_ALGORITHM_IMP_HPP
#define RAM_PATH_PLANNING_PATH_PLANNING_ALGORITHM_IMP_HPP

namespace ram_path_planning
{
template<class ActionSpec>
PathPlanningAlgorithm<ActionSpec>::PathPlanningAlgorithm(const std::string name,
    const std::string description,
    const std::string service_name) :
  name_(name),
  description_(description),
  service_name_(service_name)
{
}

template<class ActionSpec>
PathPlanningAlgorithm<ActionSpec>::~PathPlanningAlgorithm()
{
}

template<class ActionSpec>
bool PathPlanningAlgorithm<ActionSpec>::publishPercentageDone(const unsigned percentage,
    actionlib::ServerGoalHandle<ActionSpec> &gh)
{
  ActionSpec action;
  // Publish Feedback value
  if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE)
    return false;

  action.action_feedback.feedback.progress_value = percentage;
  gh.publishFeedback(action.action_feedback.feedback);
  return true;
}

template<class ActionSpec>
bool PathPlanningAlgorithm<ActionSpec>::publishStatusDone(const std::string progress_msg,
    actionlib::ServerGoalHandle<ActionSpec> &gh)
{
  ActionSpec action;
  // Publish Feedback value
  if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE)
    return false;

  action.action_feedback.feedback.progress_msg = progress_msg;
  gh.publishFeedback(action.action_feedback.feedback);
  return true;

}

template<class ActionSpec>
bool PathPlanningAlgorithm<ActionSpec>::publishStatusPercentageDone(const std::string progress_msg,
    const unsigned percentage,
    actionlib::ServerGoalHandle<ActionSpec> &gh)
{
  ActionSpec action;
  // Publish Feedback value
  if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE)
    return false;

  action.action_feedback.feedback.progress_value = percentage;
  action.action_feedback.feedback.progress_msg = progress_msg;
  gh.publishFeedback(action.action_feedback.feedback);
  return true;

}

}

#endif
