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
  if (argc < 1 + 4)
  {
    cerr << "Usage: " << argv[0] << "\033[94m" <<
        " radius \033[95m points_in_circle_arc \033[94m length \033[95m witdh " << "\033[39m" << endl;
    return 1;
  }

  const double radius(std::stof(argv[1]));
  const unsigned points_in_circle_arc(std::stof(argv[2]));
  const double length(std::stof(argv[3]));
  const double width(std::stof(argv[4]));
  cout << "Radius is " << radius << endl <<
      "Points in circle arc is " << points_in_circle_arc << endl <<
      "Length is " << length << endl <<
      "Width is " << width << endl;

  PosesPolygon polygon;
  Pose pose(Pose::Identity());

  double angle(0);
  for (unsigned i(0); i < points_in_circle_arc; ++i)
  {
    pose.translation()[0] = radius * cos(angle) + length - radius;
    pose.translation()[1] = radius * sin(angle) + width - radius;
    pose.translation()[2] = 0;
    polygon.push_back(pose);
    angle += (M_PI * 0.5) / (points_in_circle_arc - 1);
  }
  angle = M_PI * 0.5;
  for (unsigned i(0); i < points_in_circle_arc; ++i)
  {
    pose.translation()[0] = radius * cos(angle) + radius;
    pose.translation()[1] = radius * sin(angle) + width - radius;
    pose.translation()[2] = 0;
    polygon.push_back(pose);
    angle += (M_PI * 0.5) / (points_in_circle_arc - 1);
  }
  angle = M_PI;
  for (unsigned i(0); i < points_in_circle_arc; ++i)
  {
    pose.translation()[0] = radius * cos(angle) + radius;
    pose.translation()[1] = radius * sin(angle) + radius;
    pose.translation()[2] = 0;
    polygon.push_back(pose);
    angle += (M_PI * 0.5) / (points_in_circle_arc - 1);
  }
  angle = M_PI * 1.5;
  for (unsigned i(0); i < points_in_circle_arc; ++i)
  {
    pose.translation()[0] = radius * cos(angle) + length - radius;
    pose.translation()[1] = radius * sin(angle) + radius;
    pose.translation()[2] = 0;
    polygon.push_back(pose);
    angle += (M_PI * 0.5) / (points_in_circle_arc - 1);
  }

  PosesLayer layer;
  layer.push_back(polygon);
  PosesTrajectory traj;
  traj.push_back(layer);

  std::string header;
  header = "# Radius = " + std::to_string(radius) + "\n";
  header += "# Points in circle segment is = " + std::to_string(points_in_circle_arc) + "\n";
  header += "# Length is = " + std::to_string(length) + "\n";
  header += "# Width is = " + std::to_string(width) + "\n";
  writeYAMLFile(traj, "round_rectangle.yaml", true, header);
  return 0;
}
