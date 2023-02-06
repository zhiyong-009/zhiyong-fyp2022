#ifndef RAM_PATH_PLANNING_MESH_SLICER_HPP
#define RAM_PATH_PLANNING_MESH_SLICER_HPP

#include <ros/ros.h>
#include <ram_path_planning/error_observer.hpp>
#include <ram_path_planning/donghong_ding.hpp>
#include <ram_utils/file_extension.hpp>
#include <vtkActor.h>
#include <vtkCutter.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkOBJReader.h>
#include <vtkPLYReader.h>
#include <vtkPlane.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSTLReader.h>
#include <vtkStripper.h>
#include <vtkTriangleFilter.h>

namespace ram_path_planning
{
using Polygon = vtkSmartPointer<vtkPolyData>;
using PolygonVector = std::vector<Polygon>;
using Layer = std::vector<PolygonVector>;

bool readPolygonFile(const std::string file_name,
                     const vtkSmartPointer<vtkPolyData> poly_data);

unsigned sliceMesh(std::vector<Layer> &trajectory,
                   const std::string file_name,
                   const vtkSmartPointer<vtkPolyData> poly_data,
                   vtkSmartPointer<vtkStripper> &stripper,
                   const double height_between_layers,
                   const std::array<double, 3> slicing_direction,
                   const bool use_gui = false);
}

#endif
