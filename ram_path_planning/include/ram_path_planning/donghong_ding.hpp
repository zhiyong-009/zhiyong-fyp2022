#ifndef RAM_PATH_PLANNING_DONGHONG_DING_HPP
#define RAM_PATH_PLANNING_DONGHONG_DING_HPP

#include <ram_path_planning/donghong_ding_base.hpp>
#include <ram_path_planning/shemaphore.hpp>
#include <ram_utils/file_extension.hpp>
#include <ram_utils/ParseSvgFile.h>

namespace ram_path_planning
{
template<class ActionSpec>
class DonghongDing : public DonghongDingBase<ActionSpec>
{
public:

  using Polygon = vtkSmartPointer<vtkPolyData>;
  using PolygonVector = std::vector<Polygon>;
  using Layer = std::vector<PolygonVector>;

  DonghongDing();

  std::string generateOneLayerTrajectory(actionlib::ServerGoalHandle<ActionSpec> &gh,
                                         const int current_progress_value,
                                         const int next_progress_value,
                                         const Polygon poly_data,
                                         Layer &layer,
                                         const double deposited_material_width,
                                         const double contours_filtering_tolerance,
                                         const std::array<double, 3> normal_vector = {0, 0, 1},
                                         const double polygon_division_tolerance = M_PI / 6,
                                         const bool closest_to_bisector = false,
                                         const bool use_gui = false);

  std::string generateOneLayerTrajectory(actionlib::ServerGoalHandle<ActionSpec> &gh,
                                         const int current_progress_value,
                                         const int next_progress_value,
                                         const std::string file,
                                         Layer &layer,
                                         const double deposited_material_width,
                                         const double contours_filtering_tolerance,
                                         const double polygon_division_tolerance = M_PI / 6,
                                         const bool closest_to_bisector = false,
                                         const bool use_gui = false);

private:
  ros::NodeHandle nh_;

  Semaphore semaphore_;

  bool closest_to_bisector_;
  double polygon_division_tolerance_; // In radians
  double contours_filtering_tolerance_; // In meters

  bool findNotch(const Polygon poly_data,
                 vtkIdType &cell_id,
                 vtkIdType &pos,
                 double &angle);

  bool verifyAngles(const Polygon poly_data,
                    const vtkIdType notch_cell_id,
                    const vtkIdType notch_pos,
                    const vtkIdType vertex_cell_id,
                    const vtkIdType vertex_pos);

  bool intersectLineWithContours(const Polygon poly_data,
                                 double point_1[3],
                                 double point_2[3]);

  bool findVertex(const Polygon poly_data,
                  const vtkIdType notch_cell_id,
                  const vtkIdType notch_pos,
                  vtkIdType &vertex_cell_id,
                  vtkIdType &vertex_pos,
                  const double notch_angle);

  bool findIntersectWithBisector(const Polygon poly_data,
                                 const vtkIdType notch_cell_id,
                                 const vtkIdType notch_pos,
                                 vtkIdType &vertex_cell_id,
                                 vtkIdType &vertex_pos,
                                 double vertex[3]);

  bool divideInConvexPolygons(PolygonVector &polygon_source,
                              const int polygon_position,
                              const vtkSmartPointer<vtkPoints> split_points);

  double identifyZigzagDirection(const Polygon poly_data,
                                 vtkIdType &edge,
                                 vtkIdType &futhest_point);

  void identifyLeftChain(const Polygon poly_data,
                         const vtkIdType edge_id,
                         const vtkIdType opposite_point_id,
                         const vtkSmartPointer<vtkPoints> left_chain,
                         const vtkSmartPointer<vtkPoints> right_chain);

  bool offsetLeftChain(const Polygon poly_data,
                       const vtkIdType edge_id,
                       const vtkIdType opposite_point_id,
                       const vtkSmartPointer<vtkPoints> left_chain,
                       const vtkSmartPointer<vtkPoints> right_chain);

  bool zigzagGeneration(const Polygon poly_data,
                        const vtkIdType edge_id,
                        const vtkIdType opposite_point_id,
                        const vtkSmartPointer<vtkPoints> zigzag_points,
                        const double deposited_material_width);

  void mergeListOfPoints(const Polygon poly_data,
                         const vtkSmartPointer<vtkPoints> left_chain,
                         const vtkSmartPointer<vtkPoints> right_chain,
                         const vtkSmartPointer<vtkPoints> zigzag_points);

  bool mergeConvexPolygons(PolygonVector &polygon_source,
                           const vtkSmartPointer<vtkPoints> split_points,
                           const vtkIdType split_line);

  bool generateTrajectoryInConvexPolygon(const Polygon poly_data);

};

}

//include the implementation
#include <ram_path_planning/donghong_ding_imp.hpp>
#endif
