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

std::string fileExtension(const std::string full_path)
{
  size_t last_index = full_path.find_last_of("/");
  std::string file_name = full_path.substr(last_index + 1, full_path.size());

  last_index = file_name.find_last_of("\\");
  file_name = file_name.substr(last_index + 1, file_name.size());

  last_index = file_name.find_last_of(".");
  if (last_index == std::string::npos)
    return "";

  return file_name.substr(last_index + 1, file_name.size());
}

int main(int argc,
         char *argv[])
{
  if (argc < 3)
  {
    cerr << "Usage: " << argv[0] << "\033[94m"
        << " yaml_file \033[95m resize_factor" << "\033[39m" << endl;
    return 1;
  }

  std::string file_name(argv[1]);
  cout << "Source file: " << file_name << endl;

  const std::string file_extension = fileExtension(file_name);
  if (strcasecmp(file_extension.c_str(), "yaml"))
  {
    return 2;
  }
  const double resize_factor(std::stof(argv[2]));
  cout << "Resize factor is " << resize_factor << endl;

  // Load YAML file
  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();
  std::vector<unsigned> layer_count;
  if (!ram_utils::yamlFileToPolydata2(std::string(argv[1]), poly_data, layer_count))
  {
    cerr << "Could not load YAML file" << endl;
    return 2;
  }

  // Edit poly_data
  for (unsigned id(0); id < poly_data->GetPoints()->GetNumberOfPoints(); ++id)
  {
    double p[3];
    poly_data->GetPoints()->GetPoint(id, p);

    p[0] = p[0] * resize_factor;
    p[1] = p[1] * resize_factor;
    p[2] = p[2] * resize_factor;

    poly_data->GetPoints()->SetPoint(id, p);
  }

  // Edit file_name
  std::string add = "_modified";
  file_name.insert(file_name.size() - file_extension.size() - 1, "_modified");
  if (!ram_utils::polydataToYamlFile2(file_name, poly_data))
  {
    cerr << "Could not write " << file_name << endl;
    return 2;
  }

  return 0;
}
