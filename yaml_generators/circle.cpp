#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <fstream>
#include <cmath>
#include "include/ram_utils/trajectory_files_manager_1.hpp"

using namespace ram_utils;
using std::cout;
using std::endl;
using std::cerr;

int main(int argc,
         char *argv[])
{
  if (argc < 1 + 2)
  {
    cerr << "Usage: " << argv[0] << "\033[94m"
        << " radius \033[95m number_of_points" << "\033[39m" << endl;
    return 1;
  }

  const double radius(std::stof(argv[1]));
  const unsigned number_of_points(std::stof(argv[2]));
  cout << "Radius is " << radius << " and number of points is " << number_of_points << endl;

  PosesPolygon points(number_of_points);
  double angle(0);
  for (auto &point : points)
  {
    point = Pose::Identity();
    point.translation()[0] = radius * cos(angle);
    point.translation()[1] = radius * sin(angle);
    point.translation()[2] = 0;
    angle += (2 * M_PI) / number_of_points;
  }

  PosesLayer layer;
  layer.push_back(points);
  PosesTrajectory traj;
  traj.push_back(layer);

  std::string header;
  header = "# Radius = " + std::to_string(radius) + "\n";
  header += "# Number of points = " + std::to_string(number_of_points) + "\n";
  writeYAMLFile(traj, "circle.yaml", true, header);
  return 0;
}
