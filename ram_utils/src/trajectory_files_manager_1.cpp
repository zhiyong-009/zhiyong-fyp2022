#include <ram_utils/trajectory_files_manager_1.hpp>

namespace ram_utils
{

bool yamlFileToPolydata(const std::string yaml_file,
                        Polygon poly_data)
{
  // Open YAML file
  std::vector<std::vector<vtkVector3d>> polygon_point_list;
  try
  {
    YAML::Node yaml;
    yaml_parser::yamlNodeFromFileName(yaml_file, yaml);

    // Parse "layer" node in YAML file
    std::string node_name("layer");
    const YAML::Node &layer_node = yaml_parser::parseNode(yaml, node_name.c_str());
    if (!yaml[node_name])
    {
      YAML::Exception yaml_exception(YAML::Mark(), "Cannot parse " + node_name + " node");
      throw yaml_exception;
    }

    // First Loop
    // Parse all nodes in "layer" node
    for (auto iterator_node : layer_node)
    {
      // Parse "polygon" node

      std::string node_name("polygon");
      const YAML::Node &polygon_node = yaml_parser::parseNode(iterator_node, node_name.c_str());
      if (!iterator_node[node_name])
      {
        YAML::Exception iterator_node_exception(YAML::Mark(), "Cannot parse " + node_name + " node");
        throw iterator_node_exception;
      }
      std::vector<vtkVector3d> polygon;

      // Second Loop
      // Parse all "point" tags in "polygon" node
      for (auto point_node : polygon_node)
      {
        std::vector<double> point_values;
        if (!yaml_parser::parseVectorD(point_node, "point", point_values))
          continue;
        if (point_values.size() != 3)
          return false;
        vtkVector3d point(point_values[0], point_values[1], point_values[2]);
        polygon.push_back(point);
      }

      if (!polygon.empty())
        polygon_point_list.push_back(polygon);
    }
  }
  catch (const YAML::Exception &e)
  {
    std::cerr << "Problem occurred during parsing YAML file, problem is:\n" << e.msg << endl << "File name = "
        << yaml_file << std::endl;
    return false;
  }

  if (polygon_point_list.empty())
    return false;

  vtkSmartPointer<vtkCellArray> polygon_array = vtkSmartPointer<vtkCellArray>::New();
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  for (auto polygon_iterator : polygon_point_list)
  {
    // Create the polygon
    vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
    for (auto point_iterator : polygon_iterator)
    {
      polygon->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
      points->InsertNextPoint(point_iterator.GetData());
    }
    polygon_array->InsertNextCell(polygon);
  }

  poly_data->SetPolys(polygon_array);
  poly_data->SetPoints(points);
  cerr << "yamlFileToPolydata end" << endl;
  return true;
}

bool yamlFileToPolydata2(const std::string yaml_file,
                         Polygon poly_data,
                         std::vector<unsigned> &layer_count)
{
  std::vector<std::vector<vtkVector3d>> polygon_point_list;
  try
  {
    YAML::Node traj_node = YAML::LoadFile(yaml_file);

    for (const auto &node : traj_node)
    {
      for (const auto &layer_array : node)
      {
        unsigned polygon_number(0);

        YAML::Node key_layer = layer_array.first;
        if (!key_layer.IsScalar())
          continue;

        YAML::Node value_layer = layer_array.second;
        if (!value_layer.IsSequence())
          continue;

        if (key_layer.as<std::string>() != "layer")
          continue;

        for (const auto &layer : value_layer)
        {
          std::vector<vtkVector3d> vtk_polygon;
          for (const auto &polygon : layer)
          {
            YAML::Node key_polygon = polygon.first;
            if (!key_polygon.IsScalar())
              continue;

            YAML::Node value_polygon = polygon.second;
            if (!value_polygon.IsSequence())
              continue;

            if (key_polygon.as<std::string>() != "polygon")
              continue;

            unsigned i(0);
            for (const auto &pose : value_polygon)
            {
              if (!pose.IsSequence())
                continue;

              std::vector<double> p = pose.as<std::vector<double>>();
              if (p.size() != 3 && p.size() != 7)
                return false;

              vtkVector3d point(p[0], p[1], p[2]);
              vtk_polygon.push_back(point);
              ++i;
            }
            ++polygon_number;
          }
          if (!vtk_polygon.empty())
            polygon_point_list.push_back(vtk_polygon);
        }
        layer_count.push_back(polygon_number);
      }
    }
  }
  catch (YAML::Exception &e)
  {
    std::cerr << e.what() << std::endl;
    return false;
  }

  if (polygon_point_list.empty())
  {
    std::cerr << "Polygon list is empty" << std::endl;
    return false;
  }

  vtkSmartPointer<vtkCellArray> polygon_array = vtkSmartPointer<vtkCellArray>::New();
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  for (auto polygon_iterator : polygon_point_list)
  {
    // Create the polygon
    vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
    for (auto point_iterator : polygon_iterator)
    {
      polygon->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
      points->InsertNextPoint(point_iterator.GetData());
    }
    polygon_array->InsertNextCell(polygon);
  }

  poly_data->SetPolys(polygon_array);
  poly_data->SetPoints(points);
  return true;
}

bool polydataToYamlFile(const std::string yaml_file,
                        const Polygon poly_data)
{
  vtkIdType n_cells = poly_data->GetNumberOfCells();
  if (n_cells == 0)
  {
    std::cerr << "polydataToYamlFile: the polydata is empty" << std::endl;
    return false;
  }

  double p[3];
  std::ofstream yaml_file_ofstream;
  yaml_file_ofstream.open(yaml_file);
  yaml_file_ofstream << "---\n"
      "layer:\n";

  for (vtkIdType cell_id(0); cell_id < n_cells; ++cell_id)
  {
    vtkIdType n_points = poly_data->GetCell(cell_id)->GetNumberOfPoints();
    if (n_points == 0)
    {
      std::cerr << "polydataToYamlFile: the polygon " << cell_id << " in polydata is empty" << std::endl;
      return false;
    }
    yaml_file_ofstream << "  -\n"
        "    polygon:\n";
    for (vtkIdType point_id(0); point_id < n_points; ++point_id)
    {
      poly_data->GetCell(cell_id)->GetPoints()->GetPoint(point_id, p);
      yaml_file_ofstream << "    - point: [" << p[0] << ", " << p[1] << ", " << p[2] << "]\n";
    }
  }

  yaml_file_ofstream << "\n";
  yaml_file_ofstream.close();
  std::cout << "File " << yaml_file << " was written on the disk" << std::endl;
  return true;
}

bool polydataToYamlFile2(const std::string yaml_file,
                         const Polygon poly_data)
{
  vtkIdType n_cells = poly_data->GetNumberOfCells();
  if (n_cells == 0)
  {
    std::cerr << "polydataToYamlFile: the polydata is empty" << std::endl;
    return false;
  }

  double p[3];
  std::ofstream yaml_file_ofstream;
  yaml_file_ofstream.open(yaml_file);
  yaml_file_ofstream << "---\n"
      "- layer:\n";

  for (vtkIdType cell_id(0); cell_id < n_cells; ++cell_id)
  {
    vtkIdType n_points = poly_data->GetCell(cell_id)->GetNumberOfPoints();
    if (n_points == 0)
    {
      std::cerr << "polydataToYamlFile2: the polygon " << cell_id << " in polydata is empty" << std::endl;
      return false;
    }
    yaml_file_ofstream << "  - polygon:\n";
    for (vtkIdType point_id(0); point_id < n_points; ++point_id)
    {
      poly_data->GetCell(cell_id)->GetPoints()->GetPoint(point_id, p);
      yaml_file_ofstream << "    - [" << p[0] << ", " << p[1] << ", " << p[2] << "]\n";
    }
  }

  yaml_file_ofstream << "\n";
  yaml_file_ofstream.close();
  std::cout << "File " << yaml_file << " was written on the disk" << std::endl;
  return true;
}

void writeYAMLFile(const PosesTrajectory &trajectory,
                   const std::string file_name,
                   const bool no_linear,
                   const std::string header)
{
  std::ofstream yaml_file;
  yaml_file.open(file_name);
  if (!header.empty())
    yaml_file << header;
  yaml_file << "---\n";

  for (auto layer : trajectory)
  {
    yaml_file << "- layer:\n";
    for (auto polygon : layer)
    {
      yaml_file << "  - polygon:\n";
      for (auto pose : polygon)
      {
        if (no_linear)
        {
          yaml_file << "    - [" << pose.translation()[0] << ", " << pose.translation()[1] << ", "
              << pose.translation()[2] << "]\n";
        }
        else
        {
          Eigen::Quaterniond quat(pose.linear());

          yaml_file << "    - [" << pose.translation()[0] << ", " << pose.translation()[1] << ", "
              << pose.translation()[2] << ", " << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w()
              << "]\n";
        }
      }
    }
  }

  yaml_file << "\n";
  yaml_file.close();
  std::cout << "File " << file_name << " has been written" << std::endl;
}

}
