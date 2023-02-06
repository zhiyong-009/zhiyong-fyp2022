#ifndef RAM_PATH_PLANNING_DONGHONG_DINGBASE_HPP
#define RAM_PATH_PLANNING_DONGHONG_DINGBASE_HPP

#include <future>
#include <mutex>

#include <eigen_conversions/eigen_msg.h>
#include <ram_msgs/AdditiveManufacturingTrajectory.h>

#include <ram_utils/trajectory_files_manager_1.hpp>

#include <vtkCenterOfMass.h>
#include <vtkCleanPolyData.h>
#include <vtkDecimatePolylineFilter.h>
#include <vtkLine.h>
#include <vtkPlane.h>
#include <vtkPolyData.h>
#include <vtkPolygon.h>
#include <vtkSmartPointer.h>
#include <ram_path_planning/path_planning_algorithm.hpp>

namespace ram_path_planning
{
template<class ActionSpec>
  class DonghongDingBase : public PathPlanningAlgorithm<ActionSpec>
  {
  public:
    // Polygon (possibly multiple contours)
    // At the end: the polygon must contain exactly 1 contour
    using Polygon = vtkSmartPointer<vtkPolyData>;
    using PolygonVector = std::vector<Polygon>;
    using Layer = std::vector<PolygonVector>;

    DonghongDingBase(const std::string name,
                     const std::string description,
                     const std::string service_name);

    virtual ~DonghongDingBase()=0;  // Pure virtual function

    std::string connectMeshLayers(actionlib::ServerGoalHandle<ActionSpec> &gh,
                                  const int current_progress_value,
                                  const int next_progress_value,
                                  std::vector<Layer> &layers,
                                  ram_msgs::AdditiveManufacturingTrajectory &msg);

    void connectLayers(actionlib::ServerGoalHandle<ActionSpec> &gh,
                           const int current_progress_value,
                           const int next_progress_value,
                           const Layer &current_layer,
                           ram_msgs::AdditiveManufacturingTrajectory &msg,
                           const double number_of_layers,
                           const double height_between_layers,
                           const std::array<double, 3> offset_direction = {0, 0, 1});
    protected:
    double normal_vector_[3]; //Normal vector to slicing plane. [0,0,1] in YAML files
    const double calculation_tol_ = 1e-6; // In meters

    double deposited_material_width_; // In meters

    /*
     * angle in the interval [-PI , PI]
     * angleBetweenVectors(x,y) != angleBetweenVectors(y,x)
     */
    double angleBetweenVectors(const double v1[3],
                               const double v2[3]);

    void computeNormal(vtkPoints *p,
                       double *n);

    bool offsetPolygonContour(const Polygon poly_data,
                              const double deposited_material_width);

    bool intersectionBetweenContours(const Polygon poly_data);

    void identifyRelationships(const Polygon poly_data,
                               std::vector<int> &level,
                               std::vector<int> &father);

    bool organizePolygonContoursInLayer(const Polygon poly_data,
                                        const std::vector<int> level,
                                        const std::vector<int> father,
                                        Layer &layer);

    bool removeDuplicatePoints(const Polygon poly_data,
                               const double tolerance = 1e-6);

    bool mergeColinearEdges(const Polygon poly_data,
                            const double tolerance = 1e-6);

    void connectLayersWithOnePolygon(actionlib::ServerGoalHandle<ActionSpec> &gh,
                                     const int current_progress_value,
                                     const int next_progress_value,
                                     std::vector<Layer> &layers,
                                     ram_msgs::AdditiveManufacturingTrajectory &msg,
                                     const unsigned first_layer);

    void divideInLayersWithOnePolygon(std::vector<Layer> &layers,
                                      ram_msgs::AdditiveManufacturingTrajectory &msg,
                                      const unsigned first_layer);

  };

}

//include the implementation
#include <ram_path_planning/donghong_ding_base_imp.hpp>

#endif
