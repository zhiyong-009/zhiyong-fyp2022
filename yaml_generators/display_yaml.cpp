#include <iostream>
#include <string>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include "include/ram_utils/trajectory_files_manager_1.hpp"

using std::cout;
using std::cerr;
using std::endl;

int main(int argc,
         char *argv[])
{
  if (argc < 2)
  {
    cerr << "Usage: " << argv[0] << "\033[94m yaml_file" << "\033[39m" << endl;
    return 1;
  }

  // Load YAML file
  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();
  std::vector<unsigned> layer_count;
  if (!ram_utils::yamlFileToPolydata2(std::string(argv[1]), poly_data, layer_count))
    return 2;

  // Prepare display
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(poly_data);
  mapper->ScalarVisibilityOff();

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(1, 1, 1);
  actor->GetProperty()->SetLineWidth(1.0);
  actor->GetProperty()->SetRepresentation(1); // Wireframe

  // Display
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(actor);

  vtkSmartPointer<vtkRenderWindow> render_window = vtkSmartPointer<vtkRenderWindow>::New();
  render_window->AddRenderer(renderer);
  render_window->SetSize(800, 600);

  vtkSmartPointer<vtkRenderWindowInteractor> render_window_interactor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  render_window_interactor->SetRenderWindow(render_window);
  render_window_interactor->Initialize();

  vtkSmartPointer<vtkInteractorStyleTrackballCamera> image_style =
      vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
  render_window_interactor->SetInteractorStyle(image_style);

  render_window_interactor->Start();
  return 0;
}
