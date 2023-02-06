#ifndef RAM_PATH_PLANNING_REVOLVE_HPP
#define RAM_PATH_PLANNING_REVOLVE_HPP

#include <ram_path_planning/path_planning_algorithm.hpp>
#include <ram_utils/ParseSvgFile.h>
#include <unique_id/unique_id.h>

namespace ram_path_planning
{
template<class ActionSpec>
class Revolve: public PathPlanningAlgorithm<ActionSpec>
{
public:
  Revolve();

  std::string generateTrajectory(actionlib::ServerGoalHandle<ActionSpec>gh,
                                 ram_msgs::AdditiveManufacturingTrajectory &msg,
                                 const std::string file,
                                 const double deposited_material_width,
                                 const double height_between_layers,
                                 const unsigned number_of_passes,
                                 const double connection_angle,
                                 const unsigned revolution_number_of_points,
                                 const bool towards_interior,
                                 const bool slice_along_path,
                                 const unsigned arc_points);

private:
  ros::NodeHandle nh_;

  void
  addLayer(ram_msgs::AdditiveManufacturingTrajectory &msg,
           geometry_msgs::Pose p,
           const double deposited_material_width,
           const unsigned number_of_passes,
           const double connection_angle,
           const unsigned revolution_number_of_points,
           const bool towards_interior,
           const bool invert = false);

  void
  addRing(ram_msgs::AdditiveManufacturingTrajectory &msg,
          const double radius,
          const double z_height,
          const unsigned revolution_number_of_points,
          const double connection_angle,
          const unsigned layer_level);
};

}

// Include the implementation
#include <ram_path_planning/revolve_imp.hpp>
#endif
