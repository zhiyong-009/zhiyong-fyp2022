#ifndef RAM_PATH_PLANNING_CONTOURS_HPP
#define RAM_PATH_PLANNING_CONTOURS_HPP

#include <ram_path_planning/donghong_ding_base.hpp>

namespace ram_path_planning
{
template<class ActionSpec>
  class Contours : public DonghongDingBase<ActionSpec>
  {
  public:

    using Polygon = vtkSmartPointer<vtkPolyData>;
    using PolygonVector = std::vector<Polygon>;
    using Layer = std::vector<PolygonVector>;

    Contours();

    std::string generateOneLayerTrajectory(actionlib::ServerGoalHandle<ActionSpec> &gh,
                                           const Polygon poly_data,
                                           Layer &layer,
                                           const double deposited_material_width,
                                           const std::array<double, 3> normal_vector = {0, 0, 1},
                                           const bool use_gui = false);

    std::string generateOneLayerTrajectory(actionlib::ServerGoalHandle<ActionSpec> &gh,
                                           const std::string yaml_file,
                                           Layer &layer,
                                           const double deposited_material_width,
                                           const bool use_gui = false);
  };

}
//include the implementation
#include <ram_path_planning/contours_imp.hpp>
#endif
