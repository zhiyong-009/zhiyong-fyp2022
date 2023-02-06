#ifndef RAM_PATH_PLANNING_PATH_PLANNING_ALGORITHM_HPP
#define RAM_PATH_PLANNING_PATH_PLANNING_ALGORITHM_HPP

#include <ros/ros.h>
#include <actionlib/server/action_server.h>

namespace ram_path_planning
{
template<class ActionSpec>
class PathPlanningAlgorithm
{
public:
  PathPlanningAlgorithm(const std::string name,
                        const std::string description,
                        const std::string service_name);

  virtual ~PathPlanningAlgorithm() = 0;

  bool publishPercentageDone(const unsigned percentage,
                             actionlib::ServerGoalHandle<ActionSpec> &gh);

  bool publishStatusDone(const std::string progress_msg,
                         actionlib::ServerGoalHandle<ActionSpec> &gh);

  bool publishStatusPercentageDone(const std::string progress_msg,
                                   const unsigned percentage,
                                   actionlib::ServerGoalHandle<ActionSpec> &gh);

  const std::string name_;
  const std::string description_;
  const std::string service_name_;
};

}

//include the implementation
#include <ram_path_planning/path_planning_algorithm_imp.hpp>
#endif
