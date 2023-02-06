#ifndef RAM_PATH_PLANNING_CONTOURS_IMP_HPP
#define RAM_PATH_PLANNING_CONTOURS_IMP_HPP

namespace ram_path_planning
{
template<class ActionSpec>
  Contours<ActionSpec>::Contours() :
          DonghongDingBase<ActionSpec>("Contours", "Generate layers contours",
                                       "ram/path_planning/generate_trajectory/contours")

  {
    this->deposited_material_width_ = 0.002; // 2 millimeters
  }

template<class ActionSpec>
  std::string Contours<ActionSpec>::generateOneLayerTrajectory(actionlib::ServerGoalHandle<ActionSpec> &gh,
                                                               const Polygon poly_data,
                                                               Layer &layer,
                                                               const double deposited_material_width,
                                                               const std::array<double, 3> normal_vector,
                                                               const bool use_gui)
  {
    if (!poly_data)
      return "generateOneLayerTrajectory: polydata is not initialized";

    if (poly_data->GetNumberOfPoints() == 0)
      return "generateOneLayerTrajectory: polydata is empty";

    this->normal_vector_[0] = normal_vector[0];
    this->normal_vector_[1] = normal_vector[1];
    this->normal_vector_[2] = normal_vector[2];

    vtkMath::Normalize(this->normal_vector_);

    if (normal_vector[0] == 0 && normal_vector[1] == 0 && normal_vector[2] == 0)
    {
      this->normal_vector_[0] = 0;
      this->normal_vector_[1] = 0;
      this->normal_vector_[2] = 1;
    }

    this->deposited_material_width_ = deposited_material_width;

    if (!this->removeDuplicatePoints(poly_data, this->deposited_material_width_ / 2)) // tolerance default
      return "Failed to remove duplicate points";

    if (!this->mergeColinearEdges(poly_data))
      return "Failed to merge colinear edges";

    if (this->intersectionBetweenContours(poly_data))
      return "Contours intersects";

    std::vector<int> level;
    std::vector<int> father;
    this->identifyRelationships(poly_data, level, father);

    for (auto i : level)
      if (i != 0)
        return "Cannot generate a contour in a figure with holes";

    if (!this->organizePolygonContoursInLayer(poly_data, level, father, layer))
      return "Failed to organize polygon contours in layer";

    // Action feedback
    if (!this->publishStatusDone("Contours are ready to generate the trajectory", gh))
      return "GoalHandle is not active";

    for (auto polygons : layer)
      for (auto polydata : polygons)
      {
        if (!this->offsetPolygonContour(polydata, this->deposited_material_width_ / 2.0))
          return "Error in deposited material width";

        // Action feedback
        if (!this->publishStatusDone("Trajectory has been generate", gh))
          return "GoalHandle is not active";
      }

    if (use_gui)
    {
      std::string s;
      ROS_INFO_STREAM("Path generated on one layer");
      std::getline(std::cin, s);
    }
    return "";
  }

template<class ActionSpec>
  std::string Contours<ActionSpec>::generateOneLayerTrajectory(actionlib::ServerGoalHandle<ActionSpec> &gh,
                                                               const std::string yaml_file,
                                                               Layer &layer,
                                                               const double deposited_material_width,
                                                               const bool use_gui)
  {
    // Prepare contours
    const Polygon poly_data = Polygon::New();
    std::vector<unsigned> layer_count;
    if (!ram_utils::yamlFileToPolydata2(yaml_file, poly_data, layer_count))
      if (!ram_utils::yamlFileToPolydata(yaml_file, poly_data))
        return "Could not parse the YAML file";

    std::array<double, 3> normal_vector = {0, 0, 1};
    layer.clear();
    return generateOneLayerTrajectory(gh, poly_data, layer, deposited_material_width, normal_vector, use_gui);
  }

}

#endif
