#ifndef RAM_PATH_PLANNING_POLYGON_OFFSETS_HPP
#define RAM_PATH_PLANNING_POLYGON_OFFSETS_HPP

#include <algorithm>
#include <ram_path_planning/clipper/clipper.hpp>
#include <ram_path_planning/path_planning_algorithm.hpp>
#include <ram_utils/trajectory_files_manager_2.hpp>
#include <ram_utils/ParseSvgFile.h>
#include <unique_id/unique_id.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>

namespace ram_path_planning
{
template<class ActionSpec>
class PolygonOffsets: public PathPlanningAlgorithm<ActionSpec>
{
public:
  PolygonOffsets();

  std::string generateTrajectory(actionlib::ServerGoalHandle<ActionSpec>gh,
                                 ram_msgs::AdditiveManufacturingTrajectory &msg,
                                 const std::string file,
                                 const unsigned number_of_layers,
                                 const double height_between_layers,
                                 const double deposited_material_width,
                                 const double connection_value,
                                 const bool connection_type,
                                 const double offset_factor,
                                 const double safe_distance,
                                 const bool towards_interior,
                                 const unsigned number_of_passes,
                                 const unsigned join_type,
                                 const unsigned end_type,
                                 const double arc_tolerance,
                                 const double miter_limit,
                                 const unsigned arc_points,
                                 const bool discontinous_trajectory,
                                 const bool avoid_trajectories_crossing,
                                 const bool automatic_reverse_path,
                                 const bool reverse_origin_path);

private:
  bool isPathClosed(ram_msgs::AdditiveManufacturingTrajectory &msg);
  ros::NodeHandle nh_;
};

}

// Include the implementation
#include <ram_path_planning/polygon_offsets_imp.hpp>
#endif
