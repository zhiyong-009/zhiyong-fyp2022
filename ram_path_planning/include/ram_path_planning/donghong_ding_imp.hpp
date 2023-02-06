#ifndef RAM_PATH_PLANNING_DONGHONG_DING_IMP_HPP
#define RAM_PATH_PLANNING_DONGHONG_DING_IMP_HPP

namespace ram_path_planning
{
// Allow 4 concurrent threads, tweak depending on the machine you use

template<class ActionSpec>
  DonghongDing<ActionSpec>::DonghongDing() :
          DonghongDingBase<ActionSpec>("DonghongDing", "Fill volume",
                                       "ram/path_planning/generate_trajectory/donghong_ding"),
          semaphore_(4)
  {
    this->deposited_material_width_ = 0.002; // 2 millimeters

    contours_filtering_tolerance_ = 0.0025; // 2.5 millimeters
    polygon_division_tolerance_ = M_PI / 6; // 30 degrees
    closest_to_bisector_ = false;
  }

template<class ActionSpec>
  std::string DonghongDing<ActionSpec>::generateOneLayerTrajectory(actionlib::ServerGoalHandle<ActionSpec> &gh,
                                                                   const int current_progress_value,
                                                                   const int next_progress_value,
                                                                   const Polygon poly_data,
                                                                   Layer &layer,
                                                                   const double deposited_material_width,
                                                                   const double contours_filtering_tolerance,
                                                                   const std::array<double, 3> normal_vector,
                                                                   const double polygon_division_tolerance,
                                                                   const bool closest_to_bisector,
                                                                   const bool use_gui)
  {
    // Publish feedback with a percentage between current_progress_value and next_progress_value :
    // progress_value = current_progress_value + v * (next_progress_value - current_progress_value )
    // v is a parameter between 0 and 1

    float v = 0.0;
    int progress_value = current_progress_value;

    if (!poly_data)
      return "Polydata is not initialized";

    if (poly_data->GetNumberOfPoints() == 0)
      return "Polydata is empty";

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
    polygon_division_tolerance_ = polygon_division_tolerance;
    contours_filtering_tolerance_ = contours_filtering_tolerance;
    closest_to_bisector_ = closest_to_bisector;

    if (!this->removeDuplicatePoints(poly_data, this->deposited_material_width_ / 2)) // tolerance default
      return "Failed to remove duplicate points";

    if (!this->mergeColinearEdges(poly_data, contours_filtering_tolerance_))
      return "Failed to merge colinear edges";

    if (this->intersectionBetweenContours(poly_data))
      return "Contours intersects";

    std::vector<int> level;
    std::vector<int> father;
    this->identifyRelationships(poly_data, level, father);
    if (!this->organizePolygonContoursInLayer(poly_data, level, father, layer))
      return "Failed to organize polygon contours in layer";

    // Divide in convex polygons
    vtkSmartPointer<vtkPoints> split_points = vtkSmartPointer<vtkPoints>::New();
    for (unsigned i(0); i < layer.size(); ++i)
    {
      unsigned j = 0;
      while (j < layer[i].size())
      {

        if (use_gui)
        {
          std::string s;
          std::getline(std::cin, s);
          ROS_INFO_STREAM("Dividing in convex polygons");
        }

        if (this->divideInConvexPolygons(layer[i], j, split_points))
        {
          vtkIdType n_cells = layer[i][j]->GetNumberOfCells();
          if (n_cells == 1 && vtkPolygon::IsConvex(layer[i][j]->GetPoints()))
            j++;
        }
        else
          return "Error dividing the polygons";
      }
      // Action feedback
      v = 0.5 * (i + 1) / layer.size();
      progress_value = (int)(current_progress_value + v * (next_progress_value - current_progress_value));
      if (!this->publishStatusPercentageDone("Dividing concave polygon " + std::to_string(i) + " in convex polygons",
                                             progress_value,
                                             gh))
        return "GoalHandle is not active";
    }

    if (use_gui)
    {
      std::string s;
      std::getline(std::cin, s);
      ROS_INFO_STREAM("Enter was pressed: zigzag in convex polygons");
    }

    // Path generation in convex polygons
    std::vector<std::future<bool> > futures;
    for (auto polygons : layer)
      for (auto poly_data : polygons)
        futures.push_back(std::async(&DonghongDing::generateTrajectoryInConvexPolygon, this, poly_data));

    bool global_return = true;
    for (auto &t : futures)
      global_return &= t.get();
    if (!global_return)
      return "Failed to generate trajectory in one of the convex polygons";

    // Merge convex polygons. First method
    vtkIdType n_lines = split_points->GetNumberOfPoints() / 2;
    int current_line = 0;
    unsigned polygon_id = layer.size() - 1;
    for (int line_id(n_lines - 1); line_id >= 0; --line_id)  // LIFO
    {

      if (use_gui)
      {
        std::string s;
        std::getline(std::cin, s);
        ROS_INFO_STREAM("Enter was pressed: Merging polygons. Line " << line_id);
      }

      // Action feedback
      current_line++;
      v = 0.5 + 0.5 * (current_line) / (n_lines);
      progress_value = (int)(current_progress_value + v * (next_progress_value - current_progress_value));
      if (!this->publishStatusPercentageDone("Merging polygons. Line " + std::to_string(line_id), progress_value, gh))
        return "GoalHandle is not active";

      if (layer[polygon_id].size() == 1)
        polygon_id--;
      if (!mergeConvexPolygons(layer[polygon_id], split_points, line_id))
        return "Failed to merge convex polygons";
    }

    for (auto polygons : layer)
      for (auto poly_data : polygons)
      {
        if (!this->removeDuplicatePoints(poly_data))
          return "Failed to remove duplicate points";
        if (!this->mergeColinearEdges(poly_data))
          return "Failed to merge colinear edges";
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
  std::string DonghongDing<ActionSpec>::generateOneLayerTrajectory(actionlib::ServerGoalHandle<ActionSpec> &gh,
                                                                   const int current_progress_value,
                                                                   const int next_progress_value,
                                                                   const std::string file,
                                                                   Layer &layer,
                                                                   const double deposited_material_width,
                                                                   const double contours_filtering_tolerance,
                                                                   const double polygon_division_tolerance,
                                                                   const bool closest_to_bisector,
                                                                   const bool use_gui)
  {
    if (!strcasecmp(ram_utils::fileExtension(file).c_str(), "svg"))
    {
      ros::ServiceClient client = nh_.serviceClient<ram_utils::ParseSvgFile>("ram_utils/parse_svg_file");
      ram_utils::ParseSvgFile srv;
      srv.request.file_name = file;

      if (!client.call(srv))
        return "Could not call service ram_utils/parse_svg_file";

      if (!srv.response.error.empty())
        return srv.response.error;

      return "SVG files are not supported yet";
    }

    // Prepare contours
    const Polygon poly_data = Polygon::New();
    std::vector<unsigned> layer_count;
    if (!ram_utils::yamlFileToPolydata2(file, poly_data, layer_count))
      if (!ram_utils::yamlFileToPolydata(file, poly_data))
        return "Could not parse the YAML file";

    std::array<double, 3> normal_vector = {0, 0, 1};
    layer.clear();
    return generateOneLayerTrajectory(gh, current_progress_value, next_progress_value, poly_data, layer,
                                      deposited_material_width,
                                      contours_filtering_tolerance, normal_vector,
                                      polygon_division_tolerance,
                                      closest_to_bisector, use_gui);
  }

template<class ActionSpec>
  bool DonghongDing<ActionSpec>::findNotch(const Polygon poly_data,
                                           vtkIdType &cell_id,
                                           vtkIdType &pos,
                                           double &angle)
  {
    vtkIdType n_cells = poly_data->GetNumberOfCells();

    for (vtkIdType i(0); i < n_cells; ++i)
    {
      // Points in the cell i
      vtkIdType n_points = poly_data->GetCell(i)->GetNumberOfPoints();
      for (vtkIdType j(0); j < n_points; ++j)
      {
        double p0[3];
        double p1[3]; // Central point
        double p2[3];

        if (j == 0)
          poly_data->GetCell(i)->GetPoints()->GetPoint((n_points - 1), p0);
        else
          poly_data->GetCell(i)->GetPoints()->GetPoint((j - 1), p0);

        poly_data->GetCell(i)->GetPoints()->GetPoint(j, p1);
        poly_data->GetCell(i)->GetPoints()->GetPoint((j + 1) % n_points, p2);

        double v1[3]; // Vector between p1 and p0
        double v2[3]; // Vector between p1 and p2

        vtkMath::Subtract(p0, p1, v1);
        vtkMath::Subtract(p2, p1, v2);

        angle = this->angleBetweenVectors(v1, v2);
        if (angle < 0) // Found notch
        {
          cell_id = i;
          pos = j;
          return true;
        }
      }
    }

    ROS_ERROR_STREAM("findNotch: Zero notch found");
    return false;
  }

template<class ActionSpec>
  bool DonghongDing<ActionSpec>::verifyAngles(const Polygon poly_data,
                                              const vtkIdType notch_cell_id,
                                              const vtkIdType notch_pos,
                                              const vtkIdType vertex_cell_id,
                                              const vtkIdType vertex_pos)
  {
    vtkIdType n_points_cell;
    double notch_prev[3];
    double notch[3];
    double notch_next[3]; // notch next point int the polygon
    n_points_cell = poly_data->GetCell(notch_cell_id)->GetNumberOfPoints();

    if (notch_pos == 0)
      poly_data->GetCell(notch_cell_id)->GetPoints()->GetPoint((n_points_cell - 1), notch_prev);
    else
      poly_data->GetCell(notch_cell_id)->GetPoints()->GetPoint((notch_pos - 1), notch_prev);
    poly_data->GetCell(notch_cell_id)->GetPoints()->GetPoint(notch_pos, notch);
    poly_data->GetCell(notch_cell_id)->GetPoints()->GetPoint((notch_pos + 1) % n_points_cell, notch_next);

    double vertex_prev[3];
    double vertex[3];
    double vertex_next[3]; // vertex next point int the polygon
    n_points_cell = poly_data->GetCell(vertex_cell_id)->GetNumberOfPoints();

    if (vertex_pos == 0)
      poly_data->GetCell(vertex_cell_id)->GetPoints()->GetPoint((n_points_cell - 1), vertex_prev);
    else
      poly_data->GetCell(vertex_cell_id)->GetPoints()->GetPoint((vertex_pos - 1), vertex_prev);
    poly_data->GetCell(vertex_cell_id)->GetPoints()->GetPoint(vertex_pos, vertex);
    poly_data->GetCell(vertex_cell_id)->GetPoints()->GetPoint((vertex_pos + 1) % n_points_cell, vertex_next);

    // angles
    double v_1[3];
    double v_2[3];
    //angle 1: between the lines notch - notch_prev and notch - vertex
    vtkMath::Subtract(vertex, notch, v_2);
    vtkMath::Subtract(notch_prev, notch, v_1);
    if (vtkMath::AngleBetweenVectors(v_1, v_2) < polygon_division_tolerance_)
      return false;
    //angle 2: between the lines notch - notch_next and notch - vertex
    vtkMath::Subtract(notch_next, notch, v_1);
    if (vtkMath::AngleBetweenVectors(v_1, v_2) < polygon_division_tolerance_)
      return false;
    //angle 3: between the lines vertex - vertex_prev and notch - vertex
    vtkMath::Subtract(notch, vertex, v_2);
    vtkMath::Subtract(vertex_prev, vertex, v_1);
    if (vtkMath::AngleBetweenVectors(v_1, v_2) < polygon_division_tolerance_)
      return false;
    //angle 4: between the lines vertex - vertex_next and notch - vertex
    vtkMath::Subtract(vertex_next, vertex, v_1);
    if (vtkMath::AngleBetweenVectors(v_1, v_2) < polygon_division_tolerance_)
      return false;
    return true;
  }

template<class ActionSpec>
  bool DonghongDing<ActionSpec>::intersectLineWithContours(const Polygon poly_data,
                                                           double point_1[3],
                                                           double point_2[3])
  {
    vtkIdType n_cells = poly_data->GetNumberOfCells();
    for (vtkIdType cell_id(0); cell_id < n_cells; ++cell_id) //verify intersections with the geometries
    {
      vtkIdType n_edges = poly_data->GetCell(cell_id)->GetNumberOfEdges();
      for (vtkIdType edge_id(0); edge_id < n_edges; ++edge_id)
      {
        double e_p1[3];
        double e_p2[3];
        poly_data->GetCell(cell_id)->GetEdge(edge_id)->GetPoints()->GetPoint(0, e_p1);
        poly_data->GetCell(cell_id)->GetEdge(edge_id)->GetPoints()->GetPoint(1, e_p2);
        double u;
        double v;
        int intersection = 0;
        //intersect the line "notch  p_1" with the edge(line e_p1  e_p2)
        intersection = vtkLine::Intersection3D(point_1, point_2, e_p1, e_p2, u, v);
        if (intersection == 2 && std::abs(u) > this->calculation_tol_ && std::abs(u - 1) > this->calculation_tol_) // u!= 0 &&0u!=1
          return true;
      }
    }
    return false;
  }

template<class ActionSpec>
  bool DonghongDing<ActionSpec>::findVertex(const Polygon poly_data,
                                            const vtkIdType notch_cell_id,
                                            const vtkIdType notch_pos,
                                            vtkIdType &vertex_cell_id,
                                            vtkIdType &vertex_pos,
                                            const double notch_angle)
  {
    double ref_angle = notch_angle * (-1);
    vtkIdType n_cells = poly_data->GetNumberOfCells();
    bool is_vertex = false;

    //2.1. reference vectors
    double v_ref[3];

    double notch[3];
    double notch_next[3]; // notch next point int the polygon
    vtkIdType n_points_cell = poly_data->GetCell(notch_cell_id)->GetNumberOfPoints();
    poly_data->GetCell(notch_cell_id)->GetPoints()->GetPoint(notch_pos, notch);
    poly_data->GetCell(notch_cell_id)->GetPoints()->GetPoint((notch_pos + 1) % n_points_cell, notch_next);
    vtkMath::Subtract(notch, notch_next, v_ref); //vector between p2 and p1. reference vector

    //2.2. Determine whether the points are inside the zone
    bool is_notch = false; //Flag. change if the the possible second point is a notch
    float vertex_d = FLT_MAX;
    for (vtkIdType cell_id(0); cell_id < n_cells; ++cell_id)
    {
      vtkIdType n_points = poly_data->GetCell(cell_id)->GetNumberOfPoints();
      for (vtkIdType point_id(0); point_id < n_points; ++point_id)
      {
        double p0[3];
        double p[3]; // Central point
        double p2[3];
        if (point_id == 0)
          poly_data->GetCell(cell_id)->GetPoints()->GetPoint((n_points - 1), p0);
        else
          poly_data->GetCell(cell_id)->GetPoints()->GetPoint((point_id - 1), p0);
        poly_data->GetCell(cell_id)->GetPoints()->GetPoint(point_id, p);
        poly_data->GetCell(cell_id)->GetPoints()->GetPoint((point_id + 1) % n_points, p2);

        double vector[3]; // Vector between the notch and the possible second point of the line
        vtkMath::Subtract(p, notch, vector);
        double angle = this->angleBetweenVectors(v_ref, vector); // Calculate angle

        if (!(angle >= 0 && angle <= ref_angle))  // Point is not inside the reference area
          continue;
        // Measure and verify intersections with the geometries
        if (intersectLineWithContours(poly_data, notch, p))
          continue;

        // Verify if the point is a notch
        double p1_p0[3]; // Vector between p1 and p0
        double p2_p0[3]; // Vector between p1 and p2
        vtkMath::Subtract(p0, p, p1_p0);
        vtkMath::Subtract(p2, p, p2_p0);
        double point_angle = this->angleBetweenVectors(p1_p0, p2_p0);

        // Verify the flag
        if (point_angle < 0)
          if (!is_notch)
          {
            vertex_d = FLT_MAX;
            is_notch = true;
          }
        if (!((point_angle > 0 && !is_notch) || point_angle < 0))
          continue;

        double point_d; // Distance between the notch and p

        // Compute distance between the notch and p1
        if (closest_to_bisector_)
          point_d = sqrt(vtkMath::Distance2BetweenPoints(notch, p)) * std::abs(sin(angle - ref_angle / 2));
        else
          point_d = sqrt(vtkMath::Distance2BetweenPoints(notch, p));

        if (point_d < vertex_d)
        {
          // Save point
          vertex_d = point_d;
          is_vertex = true;
          vertex_pos = point_id;
          vertex_cell_id = cell_id;

        }
      }
    }

    return is_vertex;
  }

template<class ActionSpec>
  bool DonghongDing<ActionSpec>::findIntersectWithBisector(const Polygon poly_data,
                                                           const vtkIdType notch_cell_id,
                                                           const vtkIdType notch_pos,
                                                           vtkIdType &vertex_cell_id,
                                                           vtkIdType &vertex_pos,
                                                           double vertex[3])
  {
    double notch_prev[3];
    double notch[3]; // Central point
    double notch_next[3];

    vtkIdType n_points_cell = poly_data->GetCell(notch_cell_id)->GetNumberOfPoints();

    if (notch_pos == 0)
      poly_data->GetCell(notch_cell_id)->GetPoints()->GetPoint((n_points_cell - 1), notch_prev);
    else
      poly_data->GetCell(notch_cell_id)->GetPoints()->GetPoint((notch_pos - 1), notch_prev);
    poly_data->GetCell(notch_cell_id)->GetPoints()->GetPoint(notch_pos, notch);
    poly_data->GetCell(notch_cell_id)->GetPoints()->GetPoint((notch_pos + 1) % n_points_cell, notch_next);

    // Vector between p0 and p1
    double v1[3] = {notch[0] - notch_prev[0], notch[1] - notch_prev[1], notch[2] - notch_prev[2]};
    // Vector between p2 and p1. Vector reference
    double v2[3] = {notch[0] - notch_next[0], notch[1] - notch_next[1], notch[2] - notch_next[2]};
    vtkMath::Normalize(v1);
    vtkMath::Normalize(v2);
    double bisector[3] = {v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2]}; // Vector in the bisector direction
    vtkMath::Normalize(bisector);
    double bounds[6];
    poly_data->GetPoints()->GetBounds(bounds);
    double bound_min[3] = {bounds[0], bounds[2], bounds[4]};
    double bound_max[3] = {bounds[1], bounds[3], bounds[5]};

    // Maximum distance between all points in a polyData
    double d_max = sqrt(vtkMath::Distance2BetweenPoints(bound_min, bound_max));

    // Bisector: between notch and bisector_p. bisector_p = notch + d_max*bisector
    double bisector_p[3];
    for (unsigned i(0); i < 3; ++i)
      bisector_p[i] = notch[i] + d_max * bisector[i];

    // find the intersection point
    double vertex_coordinate = DBL_MAX;
    bool find_intersection = false; // flag
    vtkIdType n_cells = poly_data->GetNumberOfCells();

    for (vtkIdType cell_id(0); cell_id < n_cells; ++cell_id) // Verify intersections with the geometries
    {
      vtkIdType n_edges = poly_data->GetCell(cell_id)->GetNumberOfEdges();
      for (vtkIdType edge_id(0); edge_id < n_edges; ++edge_id)
      {
        double e_p1[3];
        double e_p2[3];
        poly_data->GetCell(cell_id)->GetEdge(edge_id)->GetPoints()->GetPoint(0, e_p1);
        poly_data->GetCell(cell_id)->GetEdge(edge_id)->GetPoints()->GetPoint(1, e_p2);
        double u;
        double v;
        int intersection = 0;
        intersection = vtkLine::Intersection3D(notch, bisector_p, e_p1, e_p2, u, v);
        if (u < vertex_coordinate && intersection == 2 && std::abs(u) > this->calculation_tol_
            && std::abs(u - 1) > this->calculation_tol_) // u!= 0 &&0u!=1
        {
          //save point
          find_intersection = true;
          vertex_coordinate = u;
          vertex_cell_id = cell_id;
          vertex_pos = edge_id;
        }
      }
    }

    if (find_intersection == false)
    {
      ROS_ERROR_STREAM("findIntersectWithBisector: Intersection not found");
      return false;
    }

    for (unsigned i = 0; i < 3; i++)
      vertex[i] = notch[i] + vertex_coordinate * d_max * bisector[i];

    return true;
  }

template<class ActionSpec>
  bool DonghongDing<ActionSpec>::divideInConvexPolygons(PolygonVector &polygon_source,
                                                        const int polygon_position,
                                                        const vtkSmartPointer<vtkPoints> split_points)
  {
    this->removeDuplicatePoints(polygon_source[polygon_position]);
    this->mergeColinearEdges(polygon_source[polygon_position]);

    vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();
    poly_data->DeepCopy(polygon_source[polygon_position]);
    vtkIdType n_cells = poly_data->GetNumberOfCells();
    if (n_cells == 0)
    {
      ROS_WARN_STREAM("divideInConvexPolygons: poly_data is empty");
      return true;
    }
    if (n_cells == 1 && vtkPolygon::IsConvex(poly_data->GetPoints()))
    {
      return true;
    }

    //1. find first notch
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    points = poly_data->GetPoints();

    vtkIdType notch_cell_id;
    vtkIdType notch_id;
    double angle;
    if (!findNotch(poly_data, notch_cell_id, notch_id, angle))
      return false;
    double notch[3];
    poly_data->GetCell(notch_cell_id)->GetPoints()->GetPoint(notch_id, notch);

    vtkIdType vertex_cell_id;
    vtkIdType vertex_pos; //position in the polygon
    vtkIdType vertex_edge_id; //edge id if this point is not a vertex --
    double vertex[3];

    //2. find second point of the dividing line (vertex)
    bool is_vertex(findVertex(poly_data, notch_cell_id, notch_id, vertex_cell_id, vertex_pos, angle));

    if (is_vertex)
      is_vertex = verifyAngles(poly_data, notch_cell_id, notch_id, vertex_cell_id, vertex_pos);
    //2.3. Eliminate notch
    if (is_vertex == false)
    {
      if (!findIntersectWithBisector(poly_data, notch_cell_id, notch_id, vertex_cell_id, vertex_edge_id, vertex))
        return false;
    }
    else
      poly_data->GetCell(vertex_cell_id)->GetPoints()->GetPoint(vertex_pos, vertex);
    //3. make the news polydatas (poly_data_a, poly_data_b)

    //    4 cases:
    //    case 1: notch and vertex are inside the same contour (polygon 0)
    //    case 2: notch is in the external contour and vertex is in a contour child
    //    case 3: notch is in a contour child and vertex is in the external contour
    //    case 4: notch and vertex are in different contour children
    // poly_data_a and poly_data_b
    vtkSmartPointer<vtkCellArray> polygon_array_a = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPoints> points_a = vtkSmartPointer<vtkPoints>::New();

    vtkSmartPointer<vtkCellArray> polygon_array_b = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPoints> points_b = vtkSmartPointer<vtkPoints>::New();

    vtkSmartPointer<vtkPoints> point_list = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkPolygon> polygon_a = vtkSmartPointer<vtkPolygon>::New();
    vtkSmartPointer<vtkPolygon> polygon_b = vtkSmartPointer<vtkPolygon>::New();

    double p[3];
    // CASE 1,
    //    -vertex_pos > notch_id (case 1.1)
    //    -vertex_pos < notch_id (case 1.2)

    // save the external contours
    point_list = poly_data->GetCell(0)->GetPoints();
    vtkIdType last_point_a;
    vtkIdType first_point_b;
    if (notch_cell_id == 0 && vertex_cell_id == 0)
    {
      if (is_vertex == true)
      {
        split_points->InsertNextPoint(notch);
        split_points->InsertNextPoint(vertex);
        last_point_a = vertex_pos;
        first_point_b = vertex_pos;
      }
      else
      {
        split_points->InsertNextPoint(vertex);
        split_points->InsertNextPoint(notch);
        last_point_a = vertex_edge_id;
        first_point_b = (vertex_edge_id + 1) % point_list->GetNumberOfPoints(); // if Vertex is between the last point and the first point
            // Save Vertex in poly_data_a
        polygon_a->GetPointIds()->InsertNextId(points_a->GetNumberOfPoints());
        polygon_a->GetPoints()->InsertNextPoint(vertex);
        points_a->InsertNextPoint(vertex);
        // Save Vertex in poly_data_b
        polygon_b->GetPointIds()->InsertNextId(points_b->GetNumberOfPoints());
        polygon_b->GetPoints()->InsertNextPoint(vertex);
        points_b->InsertNextPoint(vertex);
      }
      vtkIdType n_points = point_list->GetNumberOfPoints();
      for (vtkIdType i(0); i < n_points; ++i)
      {
        int id = (i + notch_id) % n_points;
        point_list->GetPoint(id, p);
        if (first_point_b > notch_id)
        {
          if (id >= notch_id && id <= last_point_a)
          {
            polygon_a->GetPointIds()->InsertNextId(points_a->GetNumberOfPoints());
            polygon_a->GetPoints()->InsertNextPoint(p);
            points_a->InsertNextPoint(p);
          }
          if (id >= first_point_b || id < notch_id)
          {
            polygon_b->GetPointIds()->InsertNextId(points_b->GetNumberOfPoints());
            polygon_b->GetPoints()->InsertNextPoint(p);
            points_b->InsertNextPoint(p);
          }
        }
        else if (first_point_b < notch_id)
        {
          if (id >= notch_id || (last_point_a < notch_id && id <= last_point_a))
          {
            polygon_a->GetPointIds()->InsertNextId(points_a->GetNumberOfPoints());
            polygon_a->GetPoints()->InsertNextPoint(p);
            points_a->InsertNextPoint(p);
          }

          if (id >= first_point_b && id < notch_id)
          {
            polygon_b->GetPointIds()->InsertNextId(points_b->GetNumberOfPoints());
            polygon_b->GetPoints()->InsertNextPoint(p);
            points_b->InsertNextPoint(p);
          }
        }
      }
      point_list->GetPoint(notch_id, p);
      polygon_b->GetPointIds()->InsertNextId(points_b->GetNumberOfPoints());
      polygon_b->GetPoints()->InsertNextPoint(p);
      points_b->InsertNextPoint(p);

      polygon_array_a->InsertNextCell(polygon_a);
      polygon_array_b->InsertNextCell(polygon_b);
      //save the others polygons in the polydata
      vtkSmartPointer<vtkPolygon> external_contour_a = vtkSmartPointer<vtkPolygon>::New();
      external_contour_a->DeepCopy(polygon_a);
      for (vtkIdType i = 1; i < n_cells; i++)
      {
        polygon_a = vtkSmartPointer<vtkPolygon>::New();
        polygon_b = vtkSmartPointer<vtkPolygon>::New();
        point_list = poly_data->GetCell(i)->GetPoints();
        point_list->GetPoint(0, p);
        double n[3];
        this->computeNormal(external_contour_a->GetPoints(), n);
        double bounds[6];
        external_contour_a->GetPoints()->GetBounds(bounds);

        int in_polygon_a = external_contour_a->PointInPolygon(
            p, external_contour_a->GetPoints()->GetNumberOfPoints(),
            static_cast<double*>(external_contour_a->GetPoints()->GetData()->GetVoidPointer(0)),
            bounds, n);

        if (in_polygon_a == 1) // Save the contour in poly_data_a
        {
          for (vtkIdType j = 0; j < point_list->GetNumberOfPoints(); ++j)
          {
            point_list->GetPoint(j, p);
            polygon_a->GetPointIds()->InsertNextId(points_a->GetNumberOfPoints());
            polygon_a->GetPoints()->InsertNextPoint(p);
            points_a->InsertNextPoint(p);
          }
          polygon_array_a->InsertNextCell(polygon_a);
        }
        if (in_polygon_a == 0) // Save the contour in poly_data_b
        {
          for (vtkIdType j = 0; j < point_list->GetNumberOfPoints(); ++j)
          {
            point_list->GetPoint(j, p);
            polygon_b->GetPointIds()->InsertNextId(points_b->GetNumberOfPoints());
            polygon_b->GetPoints()->InsertNextPoint(p);
            points_b->InsertNextPoint(p);
          }
          polygon_array_b->InsertNextCell(polygon_b);
        }
      }
    }
    // cases 2,3 and 4
    vtkIdType n_points;
    if (notch_cell_id != vertex_cell_id) // 1 polydata (polydata_a)
    {
      vtkIdType first_point_vertex_contour;
      if (notch_cell_id != 0 && vertex_cell_id != 0) //CASE 4
      {
        //External contour in case 4
        polygon_a = vtkSmartPointer<vtkPolygon>::New();
        point_list = poly_data->GetCell(0)->GetPoints();
        for (vtkIdType j = 0; j < point_list->GetNumberOfPoints(); ++j)
        {
          point_list->GetPoint(j, p);
          polygon_a->GetPointIds()->InsertNextId(points_a->GetNumberOfPoints());
          points_a->InsertNextPoint(p);
        }
        polygon_array_a->InsertNextCell(polygon_a);
        polygon_a = vtkSmartPointer<vtkPolygon>::New();
      }
      // notch contour
      point_list = poly_data->GetCell(notch_cell_id)->GetPoints();
      n_points = point_list->GetNumberOfPoints();
      double delta[3];

      for (vtkIdType i = 0; i <= n_points; i++)
      {
        vtkIdType pos = (i + notch_id) % n_points;
        point_list->GetPoint(pos, p);
        //modify the notch point
        if (i == 0)
        {
          double next_point[3];
          point_list->GetPoint((i + 1 + notch_id) % n_points, next_point);
          vtkMath::Subtract(next_point, p, delta);
          vtkMath::Normalize(delta);
          vtkMath::MultiplyScalar(delta, 1e-7);
          vtkMath::Add(p, delta, p);
        }
        polygon_a->GetPointIds()->InsertNextId(points_a->GetNumberOfPoints());
        polygon_a->GetPoints()->InsertNextPoint(p);
        points_a->InsertNextPoint(p);
      }
      //vertex contour
      point_list = poly_data->GetCell(vertex_cell_id)->GetPoints();
      n_points = point_list->GetNumberOfPoints();
      if (is_vertex == true)
        first_point_vertex_contour = vertex_pos;
      else
      {
        first_point_vertex_contour = (vertex_edge_id + 1) % n_points;
        polygon_a->GetPointIds()->InsertNextId(points_a->GetNumberOfPoints());
        points_a->InsertNextPoint(vertex);
      }

      for (vtkIdType i = 0; i < n_points; i++)
      {
        vtkIdType pos = (i + first_point_vertex_contour) % n_points;
        point_list->GetPoint(pos, p);
        polygon_a->GetPointIds()->InsertNextId(points_a->GetNumberOfPoints());
        points_a->InsertNextPoint(p);
      }
      polygon_a->GetPointIds()->InsertNextId(points_a->GetNumberOfPoints());
      vtkMath::Add(vertex, delta, vertex);
      points_a->InsertNextPoint(vertex);

      polygon_array_a->InsertNextCell(polygon_a);
      //save the others polygons in the polydata
      for (vtkIdType i = 1; i < n_cells; i++)
      {
        if (i != vertex_cell_id && i != notch_cell_id)
        {
          polygon_a = vtkSmartPointer<vtkPolygon>::New();
          point_list = poly_data->GetCell(i)->GetPoints();
          for (vtkIdType j = 0; j < point_list->GetNumberOfPoints(); ++j)
          {
            point_list->GetPoint(j, p);
            polygon_a->GetPointIds()->InsertNextId(points_a->GetNumberOfPoints());
            points_a->InsertNextPoint(p);
          }
          polygon_array_a->InsertNextCell(polygon_a);
        }
      }
    }
// end cases
    vtkSmartPointer<vtkPolyData> poly_data_a = vtkSmartPointer<vtkPolyData>::New();
    poly_data_a->SetPolys(polygon_array_a);
    poly_data_a->SetPoints(points_a);
    polygon_source.erase(polygon_source.begin() + polygon_position);
    polygon_source.push_back(poly_data_a);

    if (polygon_array_b->GetNumberOfCells() > 0)
    {
      vtkSmartPointer<vtkPolyData> poly_data_b = vtkSmartPointer<vtkPolyData>::New();
      poly_data_b->SetPolys(polygon_array_b);
      poly_data_b->SetPoints(points_b);
      polygon_source.push_back(poly_data_b);
    }
    return true;
  }

template<class ActionSpec>
  double DonghongDing<ActionSpec>::identifyZigzagDirection(const Polygon poly_data,
                                                           vtkIdType &edge,
                                                           vtkIdType &opposite_point_id)
  {
    if (poly_data->GetNumberOfCells() != 1)
    {
      ROS_ERROR_STREAM("identifyZigzagDirection: multiple or zero cells in the polydata");
      return -1;
    }

    vtkIdType n_points = poly_data->GetCell(0)->GetNumberOfPoints(); // number of points is equal to number of edges
    double min_global_h = DBL_MAX;

    for (vtkIdType edge_id(0); edge_id < n_points; ++edge_id)
    {
      double e_p1[3];
      double e_p2[3];
      poly_data->GetCell(0)->GetEdge(edge_id)->GetPoints()->GetPoint(0, e_p1);
      poly_data->GetCell(0)->GetEdge(edge_id)->GetPoints()->GetPoint(1, e_p2);

      double max_local_h = 0;
      vtkIdType local_opposite_point = 0; // opposite point to the edge (edge_id)
      for (vtkIdType point_id(0); point_id < n_points; ++point_id)
      {
        if (point_id != edge_id && point_id != (edge_id + 1) % n_points)
        {
          double point[3];
          poly_data->GetCell(0)->GetPoints()->GetPoint(point_id, point);
          double h = sqrt(vtkLine::DistanceToLine(point, e_p1, e_p2));

          double delta_h = h - max_local_h;
          if (delta_h > this->calculation_tol_) // h > max_local_h
          {
            max_local_h = h;
            local_opposite_point = point_id;
          }
          else if (std::abs(delta_h) < this->calculation_tol_) // two points at the same distance. Find the closest point to e_p2
          {
            double max_point[3];
            poly_data->GetCell(0)->GetPoints()->GetPoint(local_opposite_point, max_point);

            if (vtkMath::Distance2BetweenPoints(point, e_p2) < vtkMath::Distance2BetweenPoints(max_point, e_p2))
              local_opposite_point = point_id;
          }
        }
      }
      if (max_local_h < min_global_h)
      {
        min_global_h = max_local_h;
        edge = edge_id;
        opposite_point_id = local_opposite_point;
      }
    }
    return min_global_h;
  }

template<class ActionSpec>
  void DonghongDing<ActionSpec>::identifyLeftChain(const Polygon poly_data,
                                                   const vtkIdType edge_id,
                                                   const vtkIdType opposite_point_id,
                                                   const vtkSmartPointer<vtkPoints> left_chain,
                                                   const vtkSmartPointer<vtkPoints> right_chain)
  {
    vtkIdType n_points = poly_data->GetCell(0)->GetNumberOfPoints();
    // edge points :
    double e_p1[3];
    double e_p2[3];
    poly_data->GetCell(0)->GetEdge(edge_id)->GetPoints()->GetPoint(0, e_p1);
    poly_data->GetCell(0)->GetEdge(edge_id)->GetPoints()->GetPoint(1, e_p2);

    double p[3];
    poly_data->GetCell(0)->GetPoints()->GetPoint(opposite_point_id, p);
    vtkIdType last_point_id = opposite_point_id;
    vtkIdType first_point_id = (edge_id + 1) % n_points;

    poly_data->GetCell(0)->GetPoints()->GetPoint(last_point_id, p);
    right_chain->InsertNextPoint(p);

    for (vtkIdType i(0); i < n_points; ++i)
    {
      int id = (i + first_point_id) % n_points;

      poly_data->GetCell(0)->GetPoints()->GetPoint(id, p);

      if (last_point_id > first_point_id)
      {
        if (id >= first_point_id && id <= last_point_id)
          left_chain->InsertNextPoint(p);
        else
          right_chain->InsertNextPoint(p);
      }
      else
      {
        if (id < first_point_id && id > last_point_id)
          right_chain->InsertNextPoint(p);
        else
          left_chain->InsertNextPoint(p);
      }
    }
  }

template<class ActionSpec>
  bool DonghongDing<ActionSpec>::offsetLeftChain(const Polygon poly_data,
                                                 const vtkIdType edge_id,
                                                 const vtkIdType opposite_point_id,
                                                 const vtkSmartPointer<vtkPoints> left_chain,
                                                 const vtkSmartPointer<vtkPoints> right_chain)
  {
    if (poly_data->GetNumberOfCells() != 1)
    {
      ROS_ERROR_STREAM("offsetLeftChain: multiple or zero cells in the polydata");
      return false;
    }
    vtkIdType n_points = poly_data->GetCell(0)->GetNumberOfPoints();
    double e_p1[3];
    double e_p2[3];
    poly_data->GetCell(0)->GetEdge(edge_id)->GetPoints()->GetPoint(0, e_p1);
    poly_data->GetCell(0)->GetEdge(edge_id)->GetPoints()->GetPoint(1, e_p2);

    vtkIdType first_point_id = (edge_id + 1) % n_points;
    // 1. offset the polygon
    // 1.1. identify the new points
    double angle_first_point = M_PI / 2; // Because sin(angle_first_point) = 1
    vtkSmartPointer<vtkPoints> new_points = vtkSmartPointer<vtkPoints>::New();
    for (vtkIdType i(0); i < left_chain->GetNumberOfPoints(); ++i)
    {
      vtkIdType id = (first_point_id + i) % n_points;
      double p0[3];
      double p1[3]; //point
      double p2[3];

      double v1[3];
      double v2[3];
      double angle;
      double bisector[3];
      double new_point[3];

      if (id == 0)
        poly_data->GetCell(0)->GetPoints()->GetPoint((n_points - 1), p0);
      else
        poly_data->GetCell(0)->GetPoints()->GetPoint((id - 1), p0);
      poly_data->GetCell(0)->GetPoints()->GetPoint(id, p1);
      poly_data->GetCell(0)->GetPoints()->GetPoint((id + 1) % n_points, p2);

      vtkMath::Subtract(p0, p1, v1);
      vtkMath::Subtract(p2, p1, v2);
      vtkMath::Normalize(v1);
      vtkMath::Normalize(v2);
      angle = vtkMath::AngleBetweenVectors(v1, v2);
      if (i == 0)
      {
        angle_first_point = angle; // Save the value to modify the last point in the first hull
        for (int k = 0; k < 3; ++k)
          new_point[k] = p1[k] + this->deposited_material_width_ / sin(angle) * v1[k];
        if (sqrt(vtkMath::Distance2BetweenPoints(p1, new_point)) > sqrt(vtkMath::Distance2BetweenPoints(p1, p0)))
        {
          ROS_ERROR_STREAM("offsetLeftChain: error modifying the first point in the left chain");
          return false;
        }
      }
      else if (i == (left_chain->GetNumberOfPoints() - 1))
      {
        for (int k = 0; k < 3; ++k)
          new_point[k] = p1[k] + this->deposited_material_width_ / sin(angle) * v2[k];
        if (sqrt(vtkMath::Distance2BetweenPoints(p1, new_point)) > sqrt(vtkMath::Distance2BetweenPoints(p1, p2)))
        {
          ROS_ERROR_STREAM("offsetLeftChain: error modifying the last point in the left chain");
          return false;
        }
      }
      else
      {
        vtkMath::Add(v1, v2, bisector);
        vtkMath::Normalize(bisector);
        for (int k = 0; k < 3; ++k)
          new_point[k] = p1[k] + this->deposited_material_width_ / sin(angle / 2) * bisector[k];
      }
      new_points->InsertNextPoint(new_point);
    }

    // 1.2. verification, intersection among the lines
    for (vtkIdType i(0); i < left_chain->GetNumberOfPoints(); ++i)
    {
      double p_i[3];
      double new_p_i[3];
      left_chain->GetPoint(i, p_i);
      new_points->GetPoint(i, new_p_i);

      for (vtkIdType j(0); j < left_chain->GetNumberOfPoints(); ++j)
      {
        double p_j[3];
        double new_p_j[3];
        left_chain->GetPoint(i, p_j);
        new_points->GetPoint(i, new_p_j);

        double u; //  Parametric coordinate of the line 1
        double v; //  Parametric coordinate of the line 2
        int intersection = vtkLine::Intersection3D(p_i, new_p_i, p_j, new_p_j, u, v);

        if (intersection == 2)
        {
          ROS_ERROR_STREAM("offsetLeftChain: edge is too short");
          return false;
        }
      }
    }

    // 1.3. replace the points in the poly_data
    for (vtkIdType i(0); i < left_chain->GetNumberOfPoints(); ++i)
    {
      vtkIdType id = (first_point_id + i) % n_points;
      double new_point[3];
      new_points->GetPoint(i, new_point);
      poly_data->GetPoints()->SetPoint(poly_data->GetCell(0)->GetPointId(id), new_point);
      poly_data->Modified();
    }

    // 2. modify  first point in the second hull
    double p0[3]; // first point in the second hull
    double p1[3]; // opposite point in the polygon after offset
    double p2[3]; // next point to p1
    double v[3];
    right_chain->GetPoint(0, p0);
    poly_data->GetCell(0)->GetPoints()->GetPoint(opposite_point_id, p1);
    poly_data->GetCell(0)->GetPoints()->GetPoint((opposite_point_id + 1) % n_points, p2);

    if (vtkMath::Distance2BetweenPoints(p0, p1) >= vtkMath::Distance2BetweenPoints(p1, p2))
    {
      ROS_ERROR_STREAM("offsetLeftChain: Error modifying the point between the zigzag path and the right chain");
      return false;
    }

    vtkMath::Subtract(p1, p0, v);
    for (int k = 0; k < 3; ++k)
      p0[k] += 2 * v[k];
    right_chain->SetPoint(0, p0);

    // 3. add modified points in the first hull
    for (vtkIdType i(left_chain->GetNumberOfPoints() - 1); i >= 0; --i)
    {
      vtkIdType id = (first_point_id + i) % n_points;
      double p[3];
      poly_data->GetCell(0)->GetPoints()->GetPoint(id, p);
      left_chain->InsertNextPoint(p);
    }

    // 4. modify last point in the first hull
    left_chain->GetPoint((left_chain->GetNumberOfPoints() - 1), p0);
    left_chain->GetPoint((left_chain->GetNumberOfPoints() - 2), p1);
    vtkMath::Subtract(p1, p0, v);
    vtkMath::Normalize(v);
    if (sqrt(vtkMath::Distance2BetweenPoints(p0, p1)) < this->deposited_material_width_ / (sin(angle_first_point)))
    {
      ROS_ERROR_STREAM("offsetLeftChain: Error modifying the point between the left chain and the zigzag path");
      return false;
    }
    for (int k = 0; k < 3; ++k)
      p0[k] += this->deposited_material_width_ / (sin(angle_first_point)) * v[k];
    left_chain->SetPoint((left_chain->GetNumberOfPoints() - 1), p0);

    return true;
  }

template<class ActionSpec>
  bool DonghongDing<ActionSpec>::zigzagGeneration(const Polygon poly_data,
                                                  const vtkIdType edge_id,
                                                  const vtkIdType opposite_point_id,
                                                  const vtkSmartPointer<vtkPoints> zigzag_points,
                                                  const double deposited_material_width)
  {
    if (poly_data->GetNumberOfCells() != 1)
    {
      ROS_ERROR_STREAM("zigzagGeneration: multiple or zero cells in the polydata");
      return false;
    }
    vtkIdType n_edges = poly_data->GetCell(0)->GetNumberOfEdges();
    double e_p1[3];
    double e_p2[3];
    double last_p[3];
    double next_p[3];
    poly_data->GetCell(0)->GetEdge(edge_id)->GetPoints()->GetPoint(0, e_p1);
    poly_data->GetCell(0)->GetEdge(edge_id)->GetPoints()->GetPoint(1, e_p2);
    poly_data->GetPoints()->GetPoint(opposite_point_id, last_p);
    poly_data->GetPoints()->GetPoint((opposite_point_id + 1) % n_edges, next_p);
    double zigzag_dir[3]; // zigzag direction vector

    vtkMath::Subtract(e_p1, e_p2, zigzag_dir);
    vtkMath::Normalize(zigzag_dir);

    double u[3]; //vector between the first point in the edge with the zigzag direction and the opposite point
    vtkMath::Subtract(last_p, e_p2, u);
    double proj_u[3];
    vtkMath::ProjectVector(u, zigzag_dir, proj_u);
    double step_dir[3]; // step direction vector. Is perpendicular to zigzag_dir
    vtkMath::Subtract(u, proj_u, step_dir);
    double h = vtkMath::Norm(step_dir);
    vtkMath::Normalize(step_dir);

    double l = deposited_material_width;
    zigzag_points->InsertNextPoint(e_p2);
    zigzag_points->InsertNextPoint(e_p1);

    while (l <= h) // intersect the polygon with a perpendicular plane
    {

      vtkSmartPointer<vtkPoints> intersection_points = vtkSmartPointer<vtkPoints>::New();
      for (vtkIdType edge_id(0); edge_id < n_edges; ++edge_id)
      {
        double p1[3];
        double p2[3];
        poly_data->GetCell(0)->GetEdge(edge_id)->GetPoints()->GetPoint(0, p1);
        poly_data->GetCell(0)->GetEdge(edge_id)->GetPoints()->GetPoint(1, p2);
        double p0[3];
        for (unsigned k(0); k < 3; ++k)
          p0[k] = e_p2[k] + l * step_dir[k];
        double t;
        double x[3];
        int intersection = vtkPlane::IntersectWithLine(p1, p2, step_dir, p0, t, x);
        if (intersection == 1)
          intersection_points->InsertNextPoint(x);
      }

      if (intersection_points->GetNumberOfPoints() != 2)
      {
        ROS_ERROR_STREAM(
                         "zigzagGeneration: " << intersection_points->GetNumberOfPoints()
                             << " intersections. 2 expected intersections.");
        return false;
      }

      double final_point[3];
      double x_1[3];
      double x_2[3];
      zigzag_points->GetPoint((zigzag_points->GetNumberOfPoints() - 1), final_point);
      intersection_points->GetPoint(0, x_1);
      intersection_points->GetPoint(1, x_2);

      if (vtkMath::Distance2BetweenPoints(final_point, x_1) < vtkMath::Distance2BetweenPoints(final_point, x_2))
      {
        zigzag_points->InsertNextPoint(x_1);
        zigzag_points->InsertNextPoint(x_2);
      }
      else
      {
        zigzag_points->InsertNextPoint(x_2);
        zigzag_points->InsertNextPoint(x_1);
      }
      l += deposited_material_width;
    }

    // Opposite edges are parallel
    double v1[3] = {e_p1[0] - e_p2[0], e_p1[1] - e_p2[1], e_p1[2] - e_p2[2]};
    double v2[3] = {next_p[0] - last_p[0], next_p[1] - last_p[1], next_p[2] - last_p[2]};
    if (vtkMath::AngleBetweenVectors(v1, v2) < 1e-3) // Almost parallel
      zigzag_points->InsertNextPoint(next_p);
    zigzag_points->InsertNextPoint(last_p);
    return true;
  }

template<class ActionSpec>
  void DonghongDing<ActionSpec>::mergeListOfPoints(const Polygon poly_data,
                                                   const vtkSmartPointer<vtkPoints> left_chain,
                                                   const vtkSmartPointer<vtkPoints> right_chain,
                                                   const vtkSmartPointer<vtkPoints> zigzag_points)
  {
    vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
    vtkSmartPointer<vtkCellArray> polygon_array = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkPolyData> new_poly_data = vtkSmartPointer<vtkPolyData>::New();
    double p[3];
    for (vtkIdType i = 0; i < right_chain->GetNumberOfPoints(); ++i)
    {

      right_chain->GetPoint(i, p);
      polygon->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
      points->InsertNextPoint(p);
    }
    for (vtkIdType i = 0; i < left_chain->GetNumberOfPoints(); ++i)
    {

      left_chain->GetPoint(i, p);
      polygon->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
      points->InsertNextPoint(p);
    }
    for (vtkIdType i = 0; i < zigzag_points->GetNumberOfPoints(); ++i)
    {

      zigzag_points->GetPoint(i, p);
      polygon->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
      points->InsertNextPoint(p);
    }

    polygon_array->InsertNextCell(polygon);
    new_poly_data->SetPolys(polygon_array);
    new_poly_data->SetPoints(points);

    poly_data->ShallowCopy(new_poly_data);
  }

template<class ActionSpec>
  bool DonghongDing<ActionSpec>::mergeConvexPolygons(PolygonVector &polygon_source,
                                                     const vtkSmartPointer<vtkPoints> split_points,
                                                     const vtkIdType split_line)
  {
    unsigned n_polygons = polygon_source.size();

    double l0[3];
    double l1[3];
    split_points->GetPoint((2 * split_line), l0);
    split_points->GetPoint((2 * split_line + 1), l1);

// Find the two closest polygons to the split line. the distance between the split line
    unsigned polygon_1 = -1;
    vtkIdType edge_1 = -1;
    double l0_to_edge_1 = DBL_MAX;

    unsigned polygon_2 = -1;
    vtkIdType edge_2 = -1;
    double l0_to_edge_2 = DBL_MAX;

    for (unsigned polygon_id(0); polygon_id < n_polygons; ++polygon_id)
    {
      bool find_parallel = false;
      double min_distance_to_l0 = DBL_MAX;
      vtkIdType closest_edge = -1;
      vtkIdType n_edges = polygon_source[polygon_id]->GetCell(0)->GetNumberOfEdges();
      for (int edge_id(0); edge_id < n_edges; ++edge_id)
      {
        double distance_to_l0;
        double e_p1[3];
        double e_p0[3];
        polygon_source[polygon_id]->GetCell(0)->GetEdge(edge_id)->GetPoints()->GetPoint(0, e_p0);
        polygon_source[polygon_id]->GetCell(0)->GetEdge(edge_id)->GetPoints()->GetPoint(1, e_p1);
        double t;

        double v[3] = {e_p1[0] - e_p0[0], e_p1[1] - e_p0[1], e_p1[2] - e_p0[2]};
        double u[3] = {l1[0] - l0[0], l1[1] - l0[1], l1[2] - l0[2]};
        if (sin(vtkMath::AngleBetweenVectors(u, v)) < 1e-3) // the lines are almost parallel
        {
          distance_to_l0 = sqrt(vtkLine::DistanceToLine(l0, e_p0, e_p1, t));
          if (distance_to_l0 < min_distance_to_l0)
          {
            find_parallel = true;
            min_distance_to_l0 = distance_to_l0;
            closest_edge = edge_id;
          }
        }
      }
      if (find_parallel)
      {
        if (min_distance_to_l0 < l0_to_edge_1)
        {
          polygon_2 = polygon_1;
          edge_2 = edge_1;
          l0_to_edge_2 = l0_to_edge_1;

          polygon_1 = polygon_id;
          edge_1 = closest_edge;
          l0_to_edge_1 = min_distance_to_l0;
        }
        else if (min_distance_to_l0 < l0_to_edge_2)
        {
          polygon_2 = polygon_id;
          edge_2 = closest_edge;
          l0_to_edge_2 = min_distance_to_l0;
        }
      }
    }
    if (edge_1 == -1 || edge_2 == -1)
    {
      ROS_ERROR_STREAM("mergeConvexPolygons: Error finding parallels edges to the split line");
      return false;
    }

    // Find the new points
    double e1_p1[3];
    double e1_p0[3];

    double e2_p1[3];
    double e2_p0[3];

    double e1_v[3];
    double e2_v[3];

    double connection_point_1[3];
    double connection_point_2[3];

    double connection_point_3[3];
    double connection_point_4[3];

    double t1 = -1;
    double t2 = -1;
    polygon_source[polygon_1]->GetCell(0)->GetEdge(edge_1)->GetPoints()->GetPoint(0, e1_p0);
    polygon_source[polygon_1]->GetCell(0)->GetEdge(edge_1)->GetPoints()->GetPoint(1, e1_p1);

    polygon_source[polygon_2]->GetCell(0)->GetEdge(edge_2)->GetPoints()->GetPoint(0, e2_p0);
    polygon_source[polygon_2]->GetCell(0)->GetEdge(edge_2)->GetPoints()->GetPoint(1, e2_p1);

    vtkMath::Subtract(e1_p1, e1_p0, e1_v);
    vtkMath::Normalize(e1_v);
    vtkMath::Subtract(e2_p1, e2_p0, e2_v);
    vtkMath::Normalize(e2_v);

    vtkLine::DistanceToLine(l0, e2_p0, e2_p1, t2, connection_point_2);
    t2 = (t2 > 1) ? 1 : (t2 < 0) ? 0 : t2;
    vtkLine::DistanceToLine(l0, e1_p0, e1_p1, t1, connection_point_1);
    t1 = (t1 > 1) ? 1 : (t1 < 0) ? 0 : t1;

    // Find the connection points. two points in each edge
    double l; // Distance between the connection lines
    double connection_v[3];

    vtkMath::Subtract(connection_point_2, connection_point_1, connection_v);
    vtkMath::Normalize(connection_v);
    l = this->deposited_material_width_ / sin(vtkMath::AngleBetweenVectors(e1_v, connection_v));
    double e1_l = sqrt(vtkMath::Distance2BetweenPoints(e1_p0, e1_p1));
    double e2_l = sqrt(vtkMath::Distance2BetweenPoints(e2_p0, e2_p1));

    if ((t2 - l / e2_l) > 0 && t1 < 1)
    {
      for (int k = 0; k < 3; ++k)
      {
        connection_point_4[k] = connection_point_1[k] + l * e1_v[k];
        connection_point_3[k] = connection_point_2[k] - l * e2_v[k];

        if (t1 == 0 && (l - e1_l) < 0.25 * this->deposited_material_width_ && (l - e1_l) > 0) // Arbitrary value 0.25 * d
          connection_point_4[k] = e1_p1[k];
      }
    }
    else if ((t1 - l / e1_l) > 0 && t2 < 1)
    {
      for (int k = 0; k < 3; ++k)
      {
        connection_point_4[k] = connection_point_1[k];
        connection_point_3[k] = connection_point_2[k];

        connection_point_1[k] -= l * e1_v[k];
        connection_point_2[k] += l * e2_v[k];

        if (t2 == 0 && (l - e2_l) < 0.25 * this->deposited_material_width_ && (l - e2_l) > 0) // Arbitrary value
          connection_point_2[k] = e2_p1[k];
      }
    }
    else
    {
      ROS_ERROR_STREAM("mergeConvexPolygons: edge is too short");
      return false;
    }

    // Merge polygons
    vtkSmartPointer<vtkCellArray> polygon_array = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
    double p[3];

    for (vtkIdType i(0); i <= edge_1; ++i) // First polygon, part 1
    {
      polygon_source[polygon_1]->GetCell(0)->GetPoints()->GetPoint(i, p);
      polygon->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
      points->InsertNextPoint(p);
    }

    polygon->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
    points->InsertNextPoint(connection_point_1);

    polygon->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
    points->InsertNextPoint(connection_point_2);

    for (vtkIdType i(0); i < polygon_source[polygon_2]->GetNumberOfPoints(); ++i) // Second polygon,
    {
      vtkIdType id = (i + edge_2 + 1) % polygon_source[polygon_2]->GetNumberOfPoints();
      polygon_source[polygon_2]->GetCell(0)->GetPoints()->GetPoint(id, p);
      polygon->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
      points->InsertNextPoint(p);
    }

    polygon->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
    points->InsertNextPoint(connection_point_3);

    polygon->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
    points->InsertNextPoint(connection_point_4);

    for (vtkIdType i(edge_1 + 1); i < polygon_source[polygon_1]->GetNumberOfPoints(); ++i) // first polygon, part 2
    {
      polygon_source[polygon_1]->GetCell(0)->GetPoints()->GetPoint(i, p);
      polygon->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
      points->InsertNextPoint(p);
    }

    polygon_array->InsertNextCell(polygon);
    vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();
    poly_data->SetPolys(polygon_array);
    poly_data->SetPoints(points);

    if (polygon_2 > polygon_1)
    {
      polygon_source.erase(polygon_source.begin() + polygon_2);
      polygon_source.erase(polygon_source.begin() + polygon_1);
    }
    else
    {
      polygon_source.erase(polygon_source.begin() + polygon_1);
      polygon_source.erase(polygon_source.begin() + polygon_2);
    }

    polygon_source.push_back(poly_data);
    return true;
  }

template<class ActionSpec>
  bool DonghongDing<ActionSpec>::generateTrajectoryInConvexPolygon(const Polygon poly_data)
  {
    semaphore_.wait(); //Semaphore

    this->removeDuplicatePoints(poly_data);
    this->mergeColinearEdges(poly_data);

    if (!this->offsetPolygonContour(poly_data, this->deposited_material_width_ / 2.0))
    {
      semaphore_.signal();
      return false;
    };

    vtkIdType edge_id;
    vtkIdType opposite_point_id;
    double h;

    h = identifyZigzagDirection(poly_data, edge_id, opposite_point_id);
    if (h / this->deposited_material_width_ <= 4)
      ROS_WARN_STREAM("warning: h/d <= 4 | h/d = " << h / this->deposited_material_width_);

    vtkSmartPointer<vtkPoints> left_chain = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkPoints> right_chain = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkPoints> zigzag_points = vtkSmartPointer<vtkPoints>::New();

    identifyLeftChain(poly_data, edge_id, opposite_point_id, left_chain, right_chain);
    if (!offsetLeftChain(poly_data, edge_id, opposite_point_id, left_chain, right_chain))
    {
      semaphore_.signal();
      return false;
    }

    if (!this->offsetPolygonContour(poly_data, this->deposited_material_width_))
    {
      semaphore_.signal();
      return false;
    }

    if (!zigzagGeneration(poly_data, edge_id, opposite_point_id, zigzag_points, this->deposited_material_width_))
    {
      semaphore_.signal();
      return false;
    }

    mergeListOfPoints(poly_data, left_chain, right_chain, zigzag_points);
    this->removeDuplicatePoints(poly_data);
    this->mergeColinearEdges(poly_data);
    semaphore_.signal();
    return true;
  }

}

#endif
