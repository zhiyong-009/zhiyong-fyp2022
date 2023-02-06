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
  if (argc < 8)
  {
    cerr << "Usage: " << argv[0]
        << "\033[94m deposit_width \033[95m number_of_zigzags \033[94m length \033[95m angle \033[94m "
        << "number_of_layers \033[95m layer_height \033[94m invert"
        << "\033[39m" << endl;
    return 1;
  }

  const double deposit_width(std::stof(argv[1]) / 1000.0);
  const unsigned number_of_zigzags(std::stof(argv[2]));
  const double length(std::stof(argv[3]) / 1000.0);
  const double angle(std::stof(argv[4]) * M_PI / 180.0);
  const unsigned number_of_layers(std::stof(argv[5]));
  const double layer_height(std::stof(argv[6]) / 1000.0);
  const unsigned invert(std::stof(argv[7]));
  cout << "Deposit width is " << deposit_width << " meters" << endl;
  cout << "Number of zigzags is " << number_of_zigzags << endl;
  cout << "Length is " << length << " meters" << endl;
  cout << "Angle is " << angle << " rad" << endl;
  cout << "Number of layers is " << number_of_layers << endl;
  cout << "Layer height is " << layer_height << " meters" << endl;
  cout << "Invert mode is " << (invert) << endl;

  if (deposit_width <= 0)
  {
    cerr << "Deposit width must be > 0" << endl;
    return 1;
  }
  if (number_of_zigzags < 1)
  {
    cerr << "Number of zigzags must be > 0" << endl;
    return 1;
  }
  if (!(length > 0))
  {
    cerr << "Length must be > 0" << endl;
    return 1;
  }
  if (!(number_of_layers > 0))
  {
    cerr << "Number of layers must be > 0" << endl;
    return 1;
  }

  PosesTrajectory traj;
  for (unsigned i(0); i < number_of_layers; ++i)
  {
    PosesLayer layer;
    PosesPolygon points;

    Pose pose(Pose::Identity());
    if (invert == 2 && i % 2 == 1)
    {
      pose.translation().x() = length;
      pose.translation().z() = i * layer_height;
      points.push_back(pose);
      pose.translation().x() -= length;
      points.push_back(pose);

      for (unsigned j(0); j < number_of_zigzags - 1; ++j)
      {
        pose.translation().y() += deposit_width;
        points.push_back(pose);
        if (j % 2 == 0)
          pose.translation().x() += length;
        else
          pose.translation().x() -= length;
        points.push_back(pose);
      }
    }
    else
    {
      pose.translation().z() = i * layer_height;
      points.push_back(pose);
      pose.translation().x() += length;
      points.push_back(pose);

      for (unsigned j(0); j < number_of_zigzags - 1; ++j)
      {
        pose.translation().y() += deposit_width;
        points.push_back(pose);
        if (j % 2 == 0)
          pose.translation().x() -= length;
        else
          pose.translation().x() += length;
        points.push_back(pose);
      }
    }

    layer.push_back(points);
    traj.push_back(layer);
  }

  // Apply angle
  for (unsigned i(0); i < traj.size(); ++i)
  {
    const double x_offset(i * layer_height * tan(angle));
    for (auto &polygon : traj.at(i))
      for (auto &pose : polygon)
        pose.translation().y() += x_offset;
  }

  // Invert
  if (invert == 1)
  {
    for (unsigned i(0); i < traj.size(); ++i)
    {
      if (i % 2 == 0)
        continue;

      for (auto &polygon : traj.at(i))
        std::reverse(std::begin(polygon), std::end(polygon));
    }
  }

  std::string header;
  header = "# Deposit width = " + std::to_string(deposit_width) + " meters\n";
  header += "# Number of zigzags = " + std::to_string(number_of_zigzags) + "\n";
  header += "# Length = " + std::to_string(length) + " meters\n";
  header += "# Angle = " + std::to_string(angle) + " rad\n";
  header += "# Number of layers = " + std::to_string(number_of_layers) + "\n";
  header += "# Layer height = " + std::to_string(layer_height) + " meters\n";
  header += "# Invert = " + std::to_string(invert) + "\n";

  writeYAMLFile(traj, "wall.yaml", true, header);
  return 0;
}
