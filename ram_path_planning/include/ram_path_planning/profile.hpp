#ifndef RAM_PATH_PLANNING_PROFILE_HPP
#define RAM_PATH_PLANNING_PROFILE_HPP

#include <ram_path_planning/path_planning_algorithm.hpp>
#include <ram_utils/ParseSvgFile.h>
#include <unique_id/unique_id.h>

namespace ram_path_planning
{
template<class ActionSpec>
class Profile: public PathPlanningAlgorithm<ActionSpec>
{
public:
  Profile();

  std::string generateTrajectory(actionlib::ServerGoalHandle<ActionSpec>gh,
                                 ram_msgs::AdditiveManufacturingTrajectory &msg,
                                 const std::string file,
                                 const double deposited_material_width,
                                 const double height_between_layers,
                                 const unsigned number_of_passes,
                                 const bool towards_interior,
                                 const bool slice_along_path,
                                 const double angle_percentage,
                                 const unsigned angle_type,
                                 const unsigned arc_points);

private:
  ros::NodeHandle nh_;

  // height_between_layers is used to compute angle from last layer to the current layer to orient
  // the tool
  void
  addLayer(ram_msgs::AdditiveManufacturingTrajectory &msg,
           geometry_msgs::Pose p,
           const double deposited_material_width,
           const double height_between_layers,
           const unsigned number_of_passes,
           const bool towards_interior,
           const bool slice_along_path,
           const bool invert,
           const double angle_percentage,
           const unsigned angle_type);

  void
  addRing(ram_msgs::AdditiveManufacturingTrajectory &msg,
          const double radius,
          const double z_height,
          const unsigned layer_level,
          const double angle);
};

}

// Include the implementation
#include <ram_path_planning/profile_imp.hpp>
#endif
