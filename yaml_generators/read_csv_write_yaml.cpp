#include <algorithm>
#include <cctype>
#include <fstream>
#include "include/ram_utils/trajectory_files_manager_1.hpp"
#include <iostream>
#include <string>

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

int main(int argc, char *argv[])
{
  if (argc < 3)
  {
    cerr << "Usage: " << argv[0] << "\033[94m"
        << " csv_file \033[95m yaml_file" << "\033[39m" << endl;
    return 1;
  }

  std::string csv_file(argv[1]);
  cout << "CSV file: " << csv_file << endl;
  std::string yaml_file(argv[2]);
  cout << "YAML file: " << yaml_file << endl;

  std::string file_extension = fileExtension(csv_file);
  std::transform(file_extension .begin(), file_extension .end(), file_extension .begin(),
    [](unsigned char c){ return std::tolower(c); });

  if (file_extension.compare("csv"))
  {
    cerr << "Extension '" << file_extension << "' is not permitted" << endl;
    return 2;
  }

  ram_utils::PosesPolygon polygon;

  // Read CSV file
  std::ifstream file(csv_file);
  std::string str_1;
  while (std::getline(file, str_1))
  {
    size_t comma_position(str_1.find(","));
    if (comma_position == std::string::npos)
    {
      cerr << "Could not process line: " << str_1 << endl;
      continue;
    }

    std::string x_value(str_1);
    x_value.resize(comma_position);

    std::string str_2(str_1);
    str_2.erase(0, comma_position + 1);

    comma_position = str_2.find(",");
    if (comma_position == std::string::npos)
    {
      cerr << "Could not process line: " << str_1 << endl;
      continue;
    }

    std::string y_value(str_2);
    y_value.resize(comma_position);

    std::string str_3(str_2);
    str_3.erase(0, comma_position + 1);
    std::string z_value(str_3);

    float x(std::stof(x_value) / 1000.0);
    float y(std::stof(y_value) / 1000.0);
    float z(std::stof(z_value) / 1000.0);
    Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
    pose.translation() << x, y, z;
    polygon.emplace_back(pose);
  }

  // Write YAML file
  ram_utils::PosesLayer layer;
  layer.push_back(polygon);
  ram_utils::PosesTrajectory traj;
  traj.push_back(layer);

  std::string header;
  header =  "# Converted from CSV file " + csv_file + "\n";
  ram_utils::writeYAMLFile(traj, yaml_file, true, header);
  return 0;
}
