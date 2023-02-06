#include <ram_path_planning/mesh_slicer.hpp>

namespace ram_path_planning
{

bool readPolygonFile(const std::string file_name,
                     const vtkSmartPointer<vtkPolyData> poly_data)
{
  vtkSmartPointer<ErrorObserver> vtk_observer = vtkSmartPointer<ErrorObserver>::New();
  std::string file_extension = ram_utils::fileExtension(file_name);

  vtkSmartPointer<vtkAbstractPolyDataReader> reader;

  if (!strcasecmp(file_extension.c_str(), "obj"))
    reader = vtkSmartPointer<vtkOBJReader>::New();
  else if (!strcasecmp(file_extension.c_str(), "ply"))
    reader = vtkSmartPointer<vtkPLYReader>::New();
  else if (!strcasecmp(file_extension.c_str(), "stl"))
    reader = vtkSmartPointer<vtkSTLReader>::New();
  else
    return false;

  reader->SetFileName(file_name.c_str());
  reader->AddObserver(vtkCommand::WarningEvent, vtk_observer);
  reader->AddObserver(vtkCommand::ErrorEvent, vtk_observer);
  vtk_observer->Clear();
  reader->Update();

  if (vtk_observer->GetWarning())
  {
    ROS_WARN_STREAM("readPolygonFile: " << file_name << std::endl << vtk_observer->GetWarningMessage());
    vtk_observer->Clear();
    return false;
  }
  if (vtk_observer->GetError())
  {
    ROS_ERROR_STREAM("readPolygonFile: " << file_name << std::endl << vtk_observer->GetErrorMessage());
    vtk_observer->Clear();
    return false;
  }

  if (!reader->GetOutput()->GetNumberOfCells())
  {
    ROS_ERROR_STREAM("readPolygonFile: " << file_name << " has no faces!");
    return false;
  }

  poly_data->DeepCopy(reader->GetOutput());
  return true;
}

unsigned sliceMesh(std::vector<Layer> &trajectory,
                   const std::string file_name,
                   const vtkSmartPointer<vtkPolyData> poly_data,
                   vtkSmartPointer<vtkStripper> &stripper,
                   const double height_between_layers,
                   const std::array<double, 3> slicing_direction,
                   const bool use_gui)
{
  if (height_between_layers == 0)
    return 0;

  if (!poly_data)
    return 0;

  if (!readPolygonFile(file_name, poly_data))
    return 0;

  double cutter_normal[3];
  cutter_normal[0] = slicing_direction[0];
  cutter_normal[1] = slicing_direction[1];
  cutter_normal[2] = slicing_direction[2];

  if (cutter_normal[0] == 0 &&
      cutter_normal[1] == 0 &&
      cutter_normal[2] == 0)
  {
    cutter_normal[0] = 0;
    cutter_normal[1] = 0;
    cutter_normal[2] = 1;
  }

  vtkMath::Normalize(cutter_normal);

  poly_data->ComputeBounds();
  double min_bound[3];
  min_bound[0] = poly_data->GetBounds()[0];
  min_bound[1] = poly_data->GetBounds()[2];
  min_bound[2] = poly_data->GetBounds()[4];
  //ROS_INFO_STREAM("Minimum bounds: " << min_bound[0] << " " << min_bound[1] << " " << min_bound[2]);

  double max_bound[3];
  max_bound[0] = poly_data->GetBounds()[1];
  max_bound[1] = poly_data->GetBounds()[3];
  max_bound[2] = poly_data->GetBounds()[5];
  //ROS_INFO_STREAM("Maximum bounds: " << max_bound[0] << " " << max_bound[1] << " " << max_bound[2]);

  trajectory.clear();
  double min_bound_proj[3];
  double max_bound_proj[3];
  vtkMath::ProjectVector(min_bound, cutter_normal, min_bound_proj);
  vtkMath::ProjectVector(max_bound, cutter_normal, max_bound_proj);

  double min_offset = cos(vtkMath::AngleBetweenVectors(min_bound_proj, cutter_normal)) * vtkMath::Norm(min_bound_proj);
  double max_offset = cos(vtkMath::AngleBetweenVectors(max_bound_proj, cutter_normal)) * vtkMath::Norm(max_bound_proj);
  if (min_offset > max_offset)
  {
    double aux_offset = min_offset;
    min_offset = max_offset;
    max_offset = aux_offset;
  }

  double origin_offset = min_offset;
  bool first_layer(true);
  while (origin_offset < max_offset)
  {
    // Create a plane to cut mesh
    vtkSmartPointer<vtkPlane> plane = vtkSmartPointer<vtkPlane>::New();
    plane->SetOrigin(poly_data->GetCenter());
    plane->SetOrigin(0, 0, 0);
    plane->SetNormal(cutter_normal);

    // Create a cutter
    vtkSmartPointer<vtkCutter> cutter = vtkSmartPointer<vtkCutter>::New();
    cutter->SetCutFunction(plane);
    cutter->SetInputData(poly_data);

    if (first_layer)
    {
      cutter->SetValue(0, origin_offset + 1e-5);
      first_layer = false;
    }
    else
      cutter->SetValue(0, origin_offset);

    origin_offset += height_between_layers;
    cutter->Update();

    vtkSmartPointer<vtkTriangleFilter> triangle_filter = vtkSmartPointer<vtkTriangleFilter>::New();
    triangle_filter->SetInputConnection(cutter->GetOutputPort());
    triangle_filter->Update();

    stripper = vtkSmartPointer<vtkStripper>::New();
    stripper->SetInputConnection(triangle_filter->GetOutputPort());
    stripper->Update();

    Polygon poly = Polygon::New();
    poly->DeepCopy(stripper->GetOutput());
    if (poly->GetNumberOfPoints() < 3)
      ROS_INFO_STREAM("sliceMesh: Contour is empty");
    else
    {
      PolygonVector poly_vector;
      poly_vector.push_back(poly);
      Layer layer;
      layer.push_back(poly_vector);

      trajectory.push_back(layer);
      if (use_gui)
        ROS_INFO_STREAM("sliceMesh: Generated " <<poly->GetNumberOfCells()<<" contours in this layer");
    }
    if (use_gui)
    {
      ROS_INFO_STREAM("Press enter to slice the next layer");
      std::string we_dont_care;
      std::getline(std::cin, we_dont_care);
    }
  }

  if (use_gui)
    ROS_INFO_STREAM("Generated " << trajectory.size() << " slices");

  return trajectory.size();
}

}

