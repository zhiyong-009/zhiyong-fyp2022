#ifndef RAM_PATH_PLANNING_DONGHONG_DINGBASE_IMP_HPP
#define RAM_PATH_PLANNING_DONGHONG_DINGBASE_IMP_HPP

namespace ram_path_planning
{

template<class ActionSpec>
  DonghongDingBase<ActionSpec>::DonghongDingBase(const std::string name,
                                                 const std::string description,
                                                 const std::string service_name) :
          PathPlanningAlgorithm<ActionSpec>(name, description, service_name)
  {
  }

template<class ActionSpec>
  DonghongDingBase<ActionSpec>::~DonghongDingBase()
  {
  }

// Connect layers
template<class ActionSpec>
  std::string DonghongDingBase<ActionSpec>::connectMeshLayers(actionlib::ServerGoalHandle<ActionSpec> &gh,
                                                              const int current_progress_value,
                                                              const int next_progress_value,
                                                              std::vector<Layer> &layers,
                                                              ram_msgs::AdditiveManufacturingTrajectory &msg)
  {
    // 1. divide in groups with the same number of polygons
    // 2. divide this groups in layer with only one polygon
    // 3. connect the simple layers

    //std::vector<Layer> layers_vector;
    unsigned first_layer = 0;
    unsigned n_polygons = layers[0].size();

    if (n_polygons != 1)
      return "connectMeshLayers: function to connect layers with several polygons is not implemented";

    for (auto &layer : layers)
      if (layer.size() != n_polygons)
        return "connectMeshLayers: function to connect layers with several polygons is not implemented";

    connectLayersWithOnePolygon(gh, current_progress_value, next_progress_value, layers, msg, first_layer);
    return "";
  }

template<class ActionSpec>
  void DonghongDingBase<ActionSpec>::connectLayers(actionlib::ServerGoalHandle<ActionSpec> &gh,
                                                   const int current_progress_value,
                                                   const int next_progress_value,
                                                   const Layer &current_layer,
                                                   ram_msgs::AdditiveManufacturingTrajectory &msg,
                                                   const double number_of_layers,
                                                   const double height_between_layers,
                                                   const std::array<double, 3> offset_direction)
  {
    double v = 0.0;
    int progress_value = 0;

    double offset_dir[3] = {offset_direction[0], offset_direction[1], offset_direction[2]};

    if (vtkMath::Norm(offset_dir) == 0)
    {
      ROS_ERROR_STREAM("connectLayers: offset direction is not valid");
      return;
    }
    vtkMath::Normalize(offset_dir);

    unsigned layer_index(0);
    for (auto polygon_vector : current_layer)
    {
      for (auto polygon : polygon_vector)
      {
        vtkIdType n_points = polygon->GetNumberOfPoints();
        for (vtkIdType layer_id(0); layer_id < number_of_layers; ++layer_id)
        {
          double offset_vector[3];
          offset_vector[0] = height_between_layers * layer_id * offset_dir[0];
          offset_vector[1] = height_between_layers * layer_id * offset_dir[1];
          offset_vector[2] = height_between_layers * layer_id * offset_dir[2];

          for (vtkIdType point_id(0); point_id <= n_points; ++point_id)
          {
            double p[3];
            Eigen::Isometry3d pose_isometry = Eigen::Isometry3d::Identity();
            polygon->GetCell(0)->GetPoints()->GetPoint(point_id % n_points, p);

            pose_isometry.translation()[0] = (p[0] + offset_vector[0]);
            pose_isometry.translation()[1] = (p[1] + offset_vector[1]);
            pose_isometry.translation()[2] = (p[2] + offset_vector[2]);

            ram_msgs::AdditiveManufacturingPose pose_srv;
            tf::poseEigenToMsg(pose_isometry, pose_srv.pose);
            pose_srv.layer_level = layer_id;
            pose_srv.layer_index = layer_index;

            if (layer_id == 0 && point_id == 0) // First point
            {
              pose_srv.polygon_start = true;
            }
            msg.poses.push_back(pose_srv);
          }
          ++layer_index;
          v = (layer_id + 1.0) / number_of_layers;
          progress_value = (int)(current_progress_value + v * (next_progress_value - current_progress_value));
          if (!this->publishStatusPercentageDone(
              "Adding layer " + std::to_string(layer_index) + " to a trajectory message", progress_value, gh))
            return;
        }
        msg.poses.back().polygon_end = true;  // Last point
      }
    }
  }

template<class ActionSpec>
  void DonghongDingBase<ActionSpec>::connectLayersWithOnePolygon(actionlib::ServerGoalHandle<ActionSpec> &gh,
                                                                 const int current_progress_value,
                                                                 const int next_progress_value,
                                                                 std::vector<Layer> &layers,
                                                                 ram_msgs::AdditiveManufacturingTrajectory &msg,
                                                                 const unsigned first_layer)
  {
    double v = 0.0;
    int progress_value = 0;
    unsigned n_layers(layers.size());

    // layers are connected close to this line
    double l1[3]; // Point in the plane of the first layer
    double l2[3]; // Point in the last layer

    double p_first_layer[3]; // Point in the first layer

    layers.back()[0][0]->GetCell(0)->GetPoints()->GetPoint(0, l2);
    //find l1
    double l2_p[3]; // Vector between l2 and p_first layer
    vtkMath::Subtract(p_first_layer, l2, l2_p);
    double l2_l1[3]; // Vector between l2 and l1
    vtkMath::ProjectVector(l2_p, normal_vector_, l2_l1);
    vtkMath::Add(l2, l2_l1, l1); //l1

    for (unsigned layer_id(0); layer_id < n_layers; ++layer_id)
    {
      vtkIdType n_points = layers[layer_id][0][0]->GetCell(0)->GetNumberOfPoints();
      //vtkIdType closest_point_id = 0;
      vtkIdType closest_edge_id = 0;
      double connection_point[3] = {0, 0, 0};
      double shortest_distance = DBL_MAX;
      bool is_close_to_a_vertex = false;

      // Find the closest point to the line between the points l1 and l2
      for (vtkIdType id(0); id < n_points; ++id)
      {
        double e_p0[3];
        double e_p1[3];
        layers[layer_id][0][0]->GetCell(0)->GetEdge(id)->GetPoints()->GetPoint(0, e_p0);
        layers[layer_id][0][0]->GetCell(0)->GetEdge(id)->GetPoints()->GetPoint(1, e_p1);
        double closestPt1[3];
        double closestPt2[3];
        double t1;
        double t2;
        double distance_to_line = vtkLine::DistanceBetweenLineSegments(l1, l2, e_p0, e_p1, closestPt1, closestPt2, t1, t2);

        if (distance_to_line < shortest_distance)
        {
          shortest_distance = distance_to_line;

          if (sqrt(vtkMath::Distance2BetweenPoints(e_p0, closestPt2)) < deposited_material_width_ / 2) // Close to e_p0
          {
            closest_edge_id = id;

            connection_point[0] = e_p0[0];
            connection_point[1] = e_p0[1];
            connection_point[2] = e_p0[2];
            is_close_to_a_vertex = true;
          }
          else if (sqrt(vtkMath::Distance2BetweenPoints(e_p1, closestPt2)) < deposited_material_width_ / 2) // Close to e_p1
          {
            closest_edge_id = id + 1;

            connection_point[0] = e_p1[0];
            connection_point[1] = e_p1[1];
            connection_point[2] = e_p1[2];
            is_close_to_a_vertex = true;
          }
          else
          {
            closest_edge_id = id;

            connection_point[0] = closestPt2[0];
            connection_point[1] = closestPt2[1];
            connection_point[2] = closestPt2[2];
            is_close_to_a_vertex = false;
          }
        }
      }
      //save the layer points in the msg
      Eigen::Isometry3d first_pose_isometry = Eigen::Isometry3d::Identity();
      ram_msgs::AdditiveManufacturingPose first_pose_srv;

      first_pose_isometry.translation()[0] = connection_point[0];
      first_pose_isometry.translation()[1] = connection_point[1];
      first_pose_isometry.translation()[2] = connection_point[2];
      tf::poseEigenToMsg(first_pose_isometry, first_pose_srv.pose);

      first_pose_srv.layer_level = layer_id + first_layer;
      first_pose_srv.layer_index = first_pose_srv.layer_level;

      if (layer_id == 0)      // first point
        first_pose_srv.polygon_start = true;
      msg.poses.push_back(first_pose_srv);

      for (vtkIdType i(0); i < n_points; ++i)
      {
        double p[3];
        layers[layer_id][0][0]->GetCell(0)->GetPoints()->GetPoint((closest_edge_id + i + 1) % n_points, p);
        Eigen::Isometry3d pose_isometry = Eigen::Isometry3d::Identity();
        ram_msgs::AdditiveManufacturingPose pose_srv;

        pose_isometry.translation()[0] = p[0];
        pose_isometry.translation()[1] = p[1];
        pose_isometry.translation()[2] = p[2];
        tf::poseEigenToMsg(pose_isometry, pose_srv.pose);

        pose_srv.layer_level = layer_id + first_layer;
        pose_srv.layer_index = pose_srv.layer_level;

        msg.poses.push_back(pose_srv);
      }
      if (!is_close_to_a_vertex)
      {
        first_pose_srv.polygon_start = false;
        first_pose_srv.layer_level = layer_id + first_layer;
        msg.poses.push_back(first_pose_srv);
      }

      v = (layer_id + 1.0) / n_layers;
      progress_value = (int)(current_progress_value + v * (next_progress_value - current_progress_value));
      if (!this->publishStatusPercentageDone("Adding layer " + std::to_string(layer_id) + " to a trajectory message",
                                             progress_value,
                                             gh))
        return;
    }
    msg.poses.back().polygon_end = true;      // last point
  }

// Others functions
template<class ActionSpec>
  double DonghongDingBase<ActionSpec>::angleBetweenVectors(const double v1[3],
                                                           const double v2[3])
  {
    double norm = vtkMath::Norm(v1) * vtkMath::Norm(v2);

    double cross_v1_v2[3];
    vtkMath::Cross(v1, v2, cross_v1_v2);
    double sin_angle = vtkMath::Dot(normal_vector_, cross_v1_v2) / norm;
    double cos_angle = vtkMath::Dot(v1, v2) / norm;
    return atan2(sin_angle, cos_angle);
  }

template<class ActionSpec>
  void DonghongDingBase<ActionSpec>::computeNormal(vtkPoints *p,
                                                   double *n)
  {
    double ax, ay, az, bx, by, bz;

    vtkIdType n_points = p->GetNumberOfPoints();
    double p0[3];
    p->GetPoint(0, p0);
    n[0] = 0;
    n[1] = 0;
    n[2] = 0;
    for (vtkIdType i = 0; i < n_points; ++i)
    {
      double v1[3];
      p->GetPoint(i, v1);
      double v2[3];
      p->GetPoint((i + 1) % n_points, v2);
      ax = v1[0] - p0[0];
      ay = v1[1] - p0[1];
      az = v1[2] - p0[2];

      bx = v2[0] - p0[0];
      by = v2[1] - p0[1];
      bz = v2[2] - p0[2];

      // Cross Product
      n[0] += (ay * bz - az * by);
      n[1] += (az * bx - ax * bz);
      n[2] += (ax * by - ay * bx);

    }
    vtkMath::Normalize(n);
  }

template<class ActionSpec>
  bool DonghongDingBase<ActionSpec>::offsetPolygonContour(const Polygon poly_data,
                                                          const double deposited_material_width)
  {
    vtkSmartPointer<vtkCellArray> polygon_array = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkIdType n_cells = poly_data->GetNumberOfCells();
    for (vtkIdType cell_id = 0; cell_id < n_cells; ++cell_id)
    {
      vtkIdType n_points = poly_data->GetCell(cell_id)->GetNumberOfPoints();
      vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
      for (vtkIdType point_id = 0; point_id < n_points; ++point_id)
      {
        double p0[3];
        double p1[3]; // Central point
        double p2[3];
        if (point_id == 0)
          poly_data->GetCell(cell_id)->GetPoints()->GetPoint((n_points - 1), p0);
        else
          poly_data->GetCell(cell_id)->GetPoints()->GetPoint((point_id - 1), p0);
        poly_data->GetCell(cell_id)->GetPoints()->GetPoint(point_id, p1);
        poly_data->GetCell(cell_id)->GetPoints()->GetPoint((point_id + 1) % n_points, p2);
        double v2[3];
        double v1[3];
        vtkMath::Subtract(p0, p1, v1);
        vtkMath::Subtract(p2, p1, v2);
        vtkMath::Normalize(v1);
        vtkMath::Normalize(v2);
        double angle = angleBetweenVectors(v1, v2);
        double bisector[3];
        vtkMath::Add(v1, v2, bisector);
        vtkMath::Normalize(bisector);
        double new_point[3];
        for (int k = 0; k < 3; ++k)
          new_point[k] = p1[k] + deposited_material_width / sin(angle / 2) * bisector[k];

        polygon->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
        polygon->GetPoints()->InsertNextPoint(new_point);
        points->InsertNextPoint(new_point);
      }

      // First verification: intersection among the lines
      for (vtkIdType i(0); i < n_points; ++i)
      {
        double p_i[3];
        double new_p_i[3];
        poly_data->GetCell(cell_id)->GetPoints()->GetPoint(i, p_i);
        polygon->GetPoints()->GetPoint(i, new_p_i);

        for (vtkIdType j(0); j < n_points; ++j)
        {
          double p_j[3];
          double new_p_j[3];
          poly_data->GetCell(cell_id)->GetPoints()->GetPoint(j, p_j);
          polygon->GetPoints()->GetPoint(j, new_p_j);

          double u; // Parametric coordinate of the line 1
          double v; // Parametric coordinate of the line 2
          int intersection = vtkLine::Intersection3D(p_i, new_p_i, p_j, new_p_j, u, v);

          if (intersection == 2)
          {
            ROS_ERROR_STREAM("offsetPolygonContour: one or multiple edges are too short! Lines: " << i << " and " << j);
            return false;
          }
        }
      }
      polygon_array->InsertNextCell(polygon);
    }
    poly_data->SetPolys(polygon_array);
    poly_data->SetPoints(points);

    return true;
  }

template<class ActionSpec>
  bool DonghongDingBase<ActionSpec>::intersectionBetweenContours(const Polygon poly_data)
  {
    vtkIdType n_cells = poly_data->GetNumberOfCells(); // Numbers of cells in the polyData

    for (vtkIdType i(0); i < n_cells; ++i)
    {
      for (vtkIdType j(i + 1); j < n_cells; ++j)
      {
        //intersection between i contour and the (i+1=j) contour
        vtkIdType n_edges_i = poly_data->GetCell(i)->GetNumberOfEdges();
        vtkIdType n_edges_j = poly_data->GetCell(j)->GetNumberOfEdges();
        for (vtkIdType e_i(0); e_i < n_edges_i; ++e_i)
        {
          for (vtkIdType e_j(0); e_j < n_edges_j; ++e_j)
          {
            // Get the two points of the edge i
            double ei_p1[3];
            double ei_p2[3];
            poly_data->GetCell(i)->GetEdge(e_i)->GetPoints()->GetPoint(0, ei_p1);
            poly_data->GetCell(i)->GetEdge(e_i)->GetPoints()->GetPoint(1, ei_p2);
            // Get the two points of the edge j
            double ej_p1[3];
            double ej_p2[3];
            poly_data->GetCell(j)->GetEdge(e_j)->GetPoints()->GetPoint(0, ej_p1);
            poly_data->GetCell(j)->GetEdge(e_j)->GetPoints()->GetPoint(1, ej_p2);

            double u; //  Parametric coordinate of the line 1
            double v; //  Parametric coordinate of the line 2
            int intersection = vtkLine::Intersection3D(ei_p1, ei_p2, ej_p1, ej_p2, u, v);

            if (intersection == 2)
            {
              ROS_ERROR_STREAM("intersectionBetweenContours: contours " << i << " and " << j << " intersects");
              return true;
            }
            else if (intersection == 3)
            {
              // Overlapping = error
              double closest_pt1[3];
              double closest_pt2[3];
              if (vtkLine::DistanceBetweenLineSegments(ei_p1, ei_p2, ej_p1, ej_p2, closest_pt1, closest_pt2, u, v) == 0)
              {
                ROS_ERROR_STREAM("intersectionBetweenContours: contours " << i << " and " << j << " are overlapping");
                return true;
              }
            }
          }
        }
      }
    }
    return false;
  }

template<class ActionSpec>
  void DonghongDingBase<ActionSpec>::identifyRelationships(const Polygon poly_data,
                                                           std::vector<int> &level,
                                                           std::vector<int> &father)
  {
    vtkIdType n_cells = poly_data->GetNumberOfCells();

    //relationship matrix: if the j cell is inside i cell, relationship[i][j]=1
    std::vector<std::vector<int> > relationship(n_cells, std::vector<int>(n_cells, 0));

    level = std::vector<int>(n_cells, 0);
    father = std::vector<int>(n_cells, -1);

    for (vtkIdType i(0); i < n_cells; ++i)
    {
      // Points in i cell
      vtkSmartPointer<vtkPoints> point_list = vtkSmartPointer<vtkPoints>::New();
      point_list = poly_data->GetCell(i)->GetPoints();

      //Create the polygon with the points in "point_list"
      vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
      for (vtkIdType k(0); k < point_list->GetNumberOfPoints(); ++k)
      {
        double p[3];
        point_list->GetPoint(k, p);
        polygon->GetPoints()->InsertNextPoint(p[0], p[1], p[2]);
      }
      // arguments of the PointInPolygon function
      double n[3];
      computeNormal(polygon->GetPoints(), n);
      double bounds[6];
      polygon->GetPoints()->GetBounds(bounds);

      for (vtkIdType j(0); j < n_cells; ++j)
      {
        if (i != j)
        {
          //first point in the j cell
          double p[3];
          poly_data->GetCell(j)->GetPoints()->GetPoint(0, p);
          int in_polygon = polygon->PointInPolygon(
              p, polygon->GetPoints()->GetNumberOfPoints(),
              static_cast<double*>(polygon->GetPoints()->GetData()->GetVoidPointer(0)),
              bounds, n); //
          if (in_polygon)
          {
            level[j]++; // level of the polygon in the depth-tree
            relationship[i][j] = 1;
          }
        }
      }
    }
    //find the father of each contour
    for (unsigned j(0); j < n_cells; ++j)
    {
      for (unsigned i(0); i < n_cells; ++i)
      {
        if (relationship[i][j] == 1 && level[i] == (level[j] - 1)) // the level of the father cell is
        {
          father[j] = i;
          break;
        }
      }
    }

  }

template<class ActionSpec>
  bool DonghongDingBase<ActionSpec>::organizePolygonContoursInLayer(const Polygon poly_data,
                                                                    const std::vector<int> level,
                                                                    const std::vector<int> father,
                                                                    Layer &layer)
  {
    using Contour = vtkSmartPointer<vtkPolygon>;

    if (!poly_data)
    {
      ROS_ERROR_STREAM("organizePolygonContoursInLayer: poly_data is not initialized!");
      return false;
    }

    vtkIdType n_cells_contours = poly_data->GetNumberOfCells();
    if (n_cells_contours == 0)
    {
      ROS_ERROR_STREAM("organizePolygonContoursInLayer: poly_data is empty!");
      return false;
    }

    layer.clear(); // We might have deleted poly_data if it belonged to layer

    for (vtkIdType i(0); i < n_cells_contours; ++i)
    {
      if (level[i] % 2 != 0)
        continue;

      vtkSmartPointer<vtkCellArray> contour_array = vtkSmartPointer<vtkCellArray>::New();
      vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
      //save the father
      Contour contour = Contour::New();
      vtkSmartPointer<vtkPoints> point_list = vtkSmartPointer<vtkPoints>::New();
      point_list = poly_data->GetCell(i)->GetPoints(); // Points of the i cell

      // Copy all points in the new contour
      double n[3];
      computeNormal(point_list, n); // Contour orientation

      // Contour is not in the XY plane
      double angle = vtkMath::AngleBetweenVectors(n, normal_vector_);
      if (sin(angle) > calculation_tol_)
      {
        ROS_ERROR_STREAM("organizePolygonContoursInLayer: Contour is not co-planar to slicing plane ");
        return false;
      }

      if (std::abs(cos(angle) + 1) < calculation_tol_) // Clockwise direction
      {
        for (vtkIdType k(0); k < point_list->GetNumberOfPoints(); ++k)
        {
          double p[3];
          point_list->GetPoint(k, p);
          contour->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
          points->InsertNextPoint(p);
        }
      }
      else if (std::abs(cos(angle) - 1) < calculation_tol_) // Counter-clockwise direction. Reverse order
      {
        for (vtkIdType k = point_list->GetNumberOfPoints(); k > 0; --k)
        {
          double p[3];
          point_list->GetPoint((k - 1), p);
          contour->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
          points->InsertNextPoint(p);
        }
      }

      contour_array->InsertNextCell(contour);

      // Process children of the contour we just have re-ordered
      for (vtkIdType j(0); j < n_cells_contours; ++j)
      {
        if (father[j] != i)
          continue;
        //save children (father[j] == i)
        contour = vtkSmartPointer<vtkPolygon>::New();
        point_list = vtkSmartPointer<vtkPoints>::New();
        point_list = poly_data->GetCell(j)->GetPoints(); // Points of the j-th cell
        //copy all points in the new contour
        double n[3];
        computeNormal(point_list, n);          // polygon orientation
        double angle = vtkMath::AngleBetweenVectors(n, normal_vector_);
        if (sin(angle) > calculation_tol_)
        {
          ROS_ERROR_STREAM("organizePolygonContoursInLayer: Contour is not co-planar to slicing plane ");
          return false;
        }
        if (std::abs(cos(angle) - 1) < calculation_tol_)          //counter-clockwise direction
        {
          for (vtkIdType k = 0; k < point_list->GetNumberOfPoints(); k++)
          {
            double p[3];
            point_list->GetPoint(k, p);
            contour->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
            points->InsertNextPoint(p[0], p[1], p[2]);
          }
        }
        else if (std::abs(cos(angle) + 1) < calculation_tol_) //clockwise direction. Change order
        {
          for (vtkIdType k = point_list->GetNumberOfPoints(); k > 0; k--)
          {
            double p[3];
            point_list->GetPoint((k - 1), p);
            contour->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
            points->InsertNextPoint(p);
          }
        }
        else
        {
          ROS_ERROR_STREAM("organizePolygonContoursInLayer: normal vector is not parallel to Z axis");
          return false;
        }
        contour_array->InsertNextCell(contour);

      }

      // Make polygon
      Polygon polygon = Polygon::New();
      PolygonVector poly_vector;
      polygon->SetPolys(contour_array); // polygon_array contains the contours
      polygon->SetPoints(points);
      poly_vector.push_back(polygon);
      layer.push_back(poly_vector);
    }

    return true;
  }

template<class ActionSpec>
  bool DonghongDingBase<ActionSpec>::removeDuplicatePoints(const Polygon poly_data,
                                                           const double tolerance)
  {/*
   vtkSmartPointer<vtkCellArray> polygon_array = vtkSmartPointer<vtkCellArray>::New();
   vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
   vtkSmartPointer<vtkPolyData> new_poly_data = vtkSmartPointer<vtkPolyData>::New();
   vtkIdType n_cells = poly_data->GetNumberOfCells();
   for (vtkIdType cell_id(0); cell_id < n_cells; ++cell_id)
   {
   vtkIdType n_points = poly_data->GetCell(cell_id)->GetNumberOfPoints();
   vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
   for (vtkIdType point_id(0); point_id < n_points; ++point_id)
   {
   double p0[3];
   double p1[3];
   poly_data->GetCell(cell_id)->GetPoints()->GetPoint(point_id, p0);
   poly_data->GetCell(cell_id)->GetPoints()->GetPoint((point_id + 1) % n_points, p1);
   // 1. duplicate point
   if (vtkMath::Distance2BetweenPoints(p0, p1) > tolerance)
   {
   polygon->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
   points->InsertNextPoint(p0);
   }
   }
   polygon_array->InsertNextCell(polygon);
   }
   new_poly_data->SetPolys(polygon_array);
   new_poly_data->SetPoints(points);

   poly_data->ShallowCopy(new_poly_data);
   */
    vtkSmartPointer<vtkCleanPolyData> cleaner = vtkSmartPointer<vtkCleanPolyData>::New();
    cleaner->SetInputData(poly_data);
    cleaner->ToleranceIsAbsoluteOn();
    cleaner->SetAbsoluteTolerance(tolerance);
    cleaner->Update();
    if (cleaner->GetOutput()->GetNumberOfPoints() < 3)
    {
      ROS_ERROR_STREAM(
                       "removeDuplicatePoints: Not enough points" << std::endl <<
                           "vtkPolyData contains " << cleaner->GetOutput()->GetNumberOfPoints() << " points");
      return false;
    }
    poly_data->ShallowCopy(cleaner->GetOutput());
    return true;

  }

template<class ActionSpec>
  bool DonghongDingBase<ActionSpec>::mergeColinearEdges(const Polygon polygon,
                                                        const double tolerance)
  {
    vtkSmartPointer<vtkCellArray> polygon_array = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkPolyData> new_poly_data = vtkSmartPointer<vtkPolyData>::New();

    vtkIdType n_cells = polygon->GetNumberOfCells();
    for (vtkIdType cell_id(0); cell_id < n_cells; ++cell_id)
    {
      vtkIdType n_points = polygon->GetCell(cell_id)->GetNumberOfPoints();
      vtkSmartPointer<vtkPolygon> contour = vtkSmartPointer<vtkPolygon>::New();
      double p0[3];
      polygon->GetCell(cell_id)->GetPoints()->GetPoint(0, p0); //fist point in this contour
      for (vtkIdType point_id(0); point_id < n_points; ++point_id)
      {
        double p1[3]; // Central point
        double p2[3];
        polygon->GetCell(cell_id)->GetPoints()->GetPoint((point_id + 1) % n_points, p1);
        polygon->GetCell(cell_id)->GetPoints()->GetPoint((point_id + 2) % n_points, p2);

        double distance = sqrt(vtkLine::DistanceToLine(p1, p0, p2));
        if (std::abs(distance) > tolerance)
        {

          contour->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
          points->InsertNextPoint(p1);
          p0[0] = p1[0];
          p0[1] = p1[1];
          p0[2] = p1[2];
        }

      }
      if (contour->GetPointIds()->GetNumberOfIds() < 3)
      {
        ROS_ERROR_STREAM(
            "mergeColinearEdges: Cell " << cell_id << " must have at least 3 points outside the region of tolerance");

        return false;
      }
      polygon_array->InsertNextCell(contour);
    }

    new_poly_data->SetPolys(polygon_array);
    new_poly_data->SetPoints(points);

    polygon->ShallowCopy(new_poly_data);
    return true;
  }

template<class ActionSpec>
  void DonghongDingBase<ActionSpec>::divideInLayersWithOnePolygon(std::vector<Layer> &layers,
                                                                  ram_msgs::AdditiveManufacturingTrajectory &msg,
                                                                  const unsigned first_layer)
  {
    std::vector<Layer> layers_vector;
    Layer current_layer;
    unsigned n_polygons = layers[0].size();
    vtkSmartPointer<vtkCenterOfMass> center_of_mass_filter = vtkSmartPointer<vtkCenterOfMass>::New();

    while (n_polygons > 0)
    {
      double center_0[3];
      double center_1[3];
      // Compute the center of mass
      center_of_mass_filter->SetInputData(layers[0][0][0]);
      center_of_mass_filter->SetUseScalarsAsWeights(false);
      center_of_mass_filter->Update();
      center_of_mass_filter->GetCenter(center_0);

      current_layer.clear();
      layers_vector.clear();
      current_layer.push_back(layers[0][0]); //layer with only 1 polygon
      layers_vector.push_back(current_layer);

      layers[0].erase(layers[0].begin());

      for (unsigned layer_id(1); layer_id < layers.size(); ++layer_id)
      {
        unsigned closest_polygon_vector_id = 0;
        double min_distance = DBL_MAX;
        for (unsigned polygon_vector_id(1); polygon_vector_id < layers[layer_id].size(); ++polygon_vector_id)
        {
          // Compute the center of mass
          center_of_mass_filter->SetInputData(layers[layer_id][polygon_vector_id][0]);
          center_of_mass_filter->SetUseScalarsAsWeights(false);
          center_of_mass_filter->Update();
          center_of_mass_filter->GetCenter(center_1);

          double distance = sqrt(vtkMath::Distance2BetweenPoints(center_0, center_1));
          if (distance < min_distance)
          {
            min_distance = distance;
            closest_polygon_vector_id = polygon_vector_id;
          }
        }
        //
        current_layer.clear();
        current_layer.push_back(layers[layer_id][closest_polygon_vector_id]); //layer with only 1 polygon
        layers_vector.push_back(current_layer);

        layers[layer_id].erase(layers[layer_id].begin() + closest_polygon_vector_id);
      }
      //send layers
      connectLayersWithOnePolygon(layers_vector, msg, first_layer);
      n_polygons--;

    } //--- end while

  }

}

#endif
