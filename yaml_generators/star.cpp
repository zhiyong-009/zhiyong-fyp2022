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
  if (argc < 1 + 3)
  {
    cerr << "Usage: " << argv[0] << "\033[94m"
        << " inner_radius \033[95m outer_radius \033[94m number_of_branches " << "\033[39m"
        << endl;
    return 1;
  }

  const double inner_radius(std::stof(argv[1]));
  const double outer_radius(std::stof(argv[2]));
  const unsigned number_of_branches(std::stof(argv[3]));
  cout << "Inner radius is " << inner_radius << " and outer radius is " << outer_radius << endl;
  cout << "Number of branches is " << number_of_branches << endl;

  PosesPolygon outer_points(number_of_branches);
  double angle(0);
  for (auto &pose : outer_points)
  {
    pose = Pose::Identity();
    pose.translation()[0] = outer_radius * cos(angle);
    pose.translation()[1] = outer_radius * sin(angle);
    pose.translation()[2] = 0;
    angle += (2 * M_PI) / number_of_branches;
  }

  PosesPolygon inner_points(number_of_branches);
  angle = (2 * M_PI) / number_of_branches / 2.0;
  for (auto &pose : inner_points)
  {
    pose = Pose::Identity();
    pose.translation()[0] = inner_radius * cos(angle);
    pose.translation()[1] = inner_radius * sin(angle);
    pose.translation()[2] = 0;
    angle += (2 * M_PI) / number_of_branches;
  }

  // Concatenate the two vectors
  PosesPolygon polygon;
  for (unsigned i(0); i < outer_points.size(); ++i)
   {
    polygon.push_back(outer_points[i]);
    polygon.push_back(inner_points[i]);
   }

  PosesLayer layer;
  layer.push_back(polygon);
  PosesTrajectory traj;
  traj.push_back(layer);

  std::string header;
  header =  "# Inner radius = " + std::to_string(inner_radius) + "\n";
  header += "# Outer radius = " + std::to_string(outer_radius) + "\n";
  header += "# Number of branches = " + std::to_string(number_of_branches) + "\n";
  writeYAMLFile(traj, "star.yaml", true, header);
  return 0;
}
