#ifndef RAM_PATH_PLANNING_PROFILE_IMP_HPP
#define RAM_PATH_PLANNING_PROFILE_IMP_HPP

#include <Eigen/Geometry>

namespace ram_path_planning
{
template<class ActionSpec>
Profile<ActionSpec>::Profile() :
  PathPlanningAlgorithm<ActionSpec>("Profile", "Generate a revolution part using a turntable",
                                    "ram/path_planning/generate_trajectory/profile")
{
}

template<class ActionSpec>
std::string Profile<ActionSpec>::generateTrajectory(
  actionlib::ServerGoalHandle<ActionSpec>,
  ram_msgs::AdditiveManufacturingTrajectory &msg,
  const std::string file,
  const double deposited_material_width,
  const double height_between_layers,
  const unsigned number_of_passes,
  const bool towards_interior,
  const bool slice_along_path,
  const double angle_percentage,
  const unsigned angle_type,
  const unsigned arc_points)
{
  msg.file = file;
  msg.generated = ros::Time::now();
  msg.modified = msg.generated;
  msg.poses.clear();
  msg.similar_layers = false;

  // Read SVG file
  ros::ServiceClient client = nh_.serviceClient<ram_utils::ParseSvgFile>("ram_utils/parse_svg_file");
  ram_utils::ParseSvgFile srv;
  srv.request.file_name = file;
  srv.request.arc_points = arc_points;

  if (!client.call(srv))
    return "Could not call service ram_utils/parse_svg_file";

  if (!srv.response.error.empty())
    return srv.response.error;

  if (srv.response.number_of_paths != 1)
    return "Only one path is allowed in the SVG file. File has " + std::to_string(srv.response.number_of_paths) + " paths.";

  if (srv.response.traj.poses.size() < 3)
    return "Trajectory is too small, at least 2 segments are required in the drawing.";

  ram_msgs::AdditiveManufacturingTrajectory svg_traj = srv.response.traj;

  // If last pose if closer to zero than first pose, reverse the trajectory.
  {
    const double first_pose_squared_distance
    (std::pow(svg_traj.poses.front().pose.position.x, 2) + std::pow(svg_traj.poses.front().pose.position.y, 2));

    const double last_pose_squared_distance
    (std::pow(svg_traj.poses.back().pose.position.x, 2) + std::pow(svg_traj.poses.back().pose.position.y, 2));

    if (first_pose_squared_distance > last_pose_squared_distance)
      std::reverse(std::begin(svg_traj.poses), std::end(svg_traj.poses));
  }

  const double x_offset(svg_traj.poses.front().pose.position.x);
  const double y_offset(svg_traj.poses.front().pose.position.y);
  svg_traj.poses.erase(std::begin(svg_traj.poses));

  std::size_t i(0);
  for (auto &ram_pose : svg_traj.poses)
  {
    ram_pose.pose.position.x -= x_offset;
    ram_pose.pose.position.y -= y_offset;

    if (ram_pose.pose.position.x < 0)
      return "One of the pose has a X coordinate below zero. Point " + std::to_string(i) +
             " X = " + std::to_string(ram_pose.pose.position.x);

    ++i;
  }

  if (svg_traj.poses.front().pose.position.y != 0.0)
    return "The first pose of the profile must have a Y = 0 coordinate. Y = " + std::to_string(svg_traj.poses.front().pose.position.y);

  {
    // Make sure that all points in the profile are always going in the same direction (up).
    ram_msgs::AdditiveManufacturingPose last_pose;
    bool first(true);
    for (auto ram_pose : svg_traj.poses)
    {
      if (first == true)
      {
        first = false;
        last_pose = ram_pose;
        continue;
      }

      if (ram_pose.pose.position.y - last_pose.pose.position.y == 0)
        return "The profile cannot be horizontal.";
      if (ram_pose.pose.position.y - last_pose.pose.position.y < 0)
        return "The profile must be continuously in the same direction.";
      last_pose = ram_pose;
    }
  }

  if (slice_along_path)
  {
    // Switch from XY (drawing) to XZ (trajectory)
    for (auto &ram_pose : svg_traj.poses)
    {
      ram_pose.pose.position.z = ram_pose.pose.position.y;
      ram_pose.pose.position.y = 0;
    }

    i = 0;
    geometry_msgs::Pose last_pose(svg_traj.poses.front().pose);
    try
    {
      addLayer(msg, last_pose, deposited_material_width, height_between_layers, number_of_passes,
               towards_interior, slice_along_path, i % 2, angle_percentage, angle_type);
    }
    catch (std::runtime_error &e)
    {
      return e.what();
    }
    ++i;

    std::vector<ram_msgs::AdditiveManufacturingPose>::iterator it(svg_traj.poses.begin());
    ++it;
    while (1)
    {
      geometry_msgs::Pose next_pose;

      // Segment [AB], A is the first pose, B is the second one
      const geometry_msgs::Pose pose_a((it - 1)->pose);
      const geometry_msgs::Pose pose_b(it->pose);

      // If line is vertical don't try to find the ax+b equation
      if (pose_a.position.x == pose_b.position.x)
      {
        if (last_pose.position.z + height_between_layers > pose_b.position.z)
        {
          // Point would be out of the segment, look for the next segment
          if (++it == svg_traj.poses.end())
            break; // If there is no further segment, we are done
          continue;
        }

        last_pose.position.z += height_between_layers;
        next_pose = last_pose;
        try
        {
          addLayer(msg, last_pose, deposited_material_width, height_between_layers, number_of_passes,
                   towards_interior, slice_along_path, i % 2, angle_percentage, angle_type);
        }
        catch (std::runtime_error &e)
        {
          return e.what();
        }
        ++i;
        continue;
      }

      // Line is NOT vertical,c ompute line equation
      // Equation of the line f(x) = a*x + b
      const double line_coeff_a((pose_b.position.z - pose_a.position.z) / (pose_b.position.x - pose_a.position.x));
      const double line_coeff_b(pose_a.position.z - (line_coeff_a * pose_a.position.x));

      // Find the intersections of the circle and the line:
      // Zero intersections: should never happen, something went wrong!
      // One intersection: line is tangent to the circle (should be VERY rare)
      // Two intersections: most common case
      double discriminant(0);

      // Equation: ax² + bx + c = 0
      const double a(1 + std::pow(line_coeff_a, 2));
      const double b((2 * line_coeff_a * line_coeff_b) - (2 * last_pose.position.x) - (2 * line_coeff_a * last_pose.position.z));
      const double c(std::pow(last_pose.position.x, 2) + std::pow(last_pose.position.z - line_coeff_b, 2) - std::pow(height_between_layers, 2));

      discriminant = std::pow(b, 2) - 4 * a * c; // b² - 4ac

      if (discriminant < 0)
        return "Negative discriminant when searching for circle intersections.";

      double x1, x2, y1, y2;
      x1 = (-b - std::sqrt(discriminant)) / (2 * a);
      x2 = (-b + std::sqrt(discriminant)) / (2 * a);

      y1 = line_coeff_a * x1 + line_coeff_b; // z1
      y2 = line_coeff_a * x2 + line_coeff_b; // z2

      // Keep solution that has the greatest Y coordinate
      // If discrimant = 0 then the two solutions are identical
      Eigen::Vector2d C; // intersection
      if (y1 > y2)
      {
        C.x() = x1;
        C.y() = y1;
      }
      else
      {
        C.x() = x2;
        C.y() = y2;
      }

      // Check that this solution is within the current segment
      const Eigen::Vector2d A(pose_a.position.x, pose_a.position.z);
      const Eigen::Vector2d B(pose_b.position.x, pose_b.position.z);

      const Eigen::Vector2d AC(C - A);
      const Eigen::Vector2d BC(C - B);

      // Tolerance to allow creating points just a tiny bit outside the segment, this makes sure we will never get
      // discrimants < 0 when trying to find the circle intersection.
      if (AC.dot(BC) > 1e-8)
      {
        // The intersection does NOT belong to the segment
        if (++it == svg_traj.poses.end())
          break; // If there is no further segment, we are done
        continue;
      }

      last_pose.position.x = C.x();
      last_pose.position.z = C.y();
      next_pose = last_pose;
      try
      {
        addLayer(msg, last_pose, deposited_material_width, height_between_layers, number_of_passes,
                 towards_interior, slice_along_path, i % 2, angle_percentage, angle_type);
      }
      catch (std::runtime_error &e)
      {
        return e.what();
      }
      ++i;
    }
  }
  else
  {
    // Slice on the Y axis (of the drawing) to generate Z layers (for the trajectory)
    double current_y_slice(0);
    i = 0;
    std::vector<ram_msgs::AdditiveManufacturingPose>::iterator it(svg_traj.poses.begin());
    ++it;
    while (it != svg_traj.poses.end())
    {
      while (it != svg_traj.poses.end() && it->pose.position.y < current_y_slice)
        ++it;

      if (it == svg_traj.poses.begin())
        return "Error while determining the slices";

      const geometry_msgs::Pose low_pose((it - 1)->pose);
      const geometry_msgs::Pose high_pose(it->pose);

      geometry_msgs::Pose p;
      p.orientation.x = p.orientation.y = p.orientation.z;
      p.orientation.w = 1;

      // If line is vertical don't try to find the ax+b equation
      if (high_pose.position.x == low_pose.position.x)
      {
        p.position.x = high_pose.position.x;
        p.position.z = current_y_slice; // Along Z axis
      }
      else
      {
        // Equation of the line f(x) = a*x + b
        const double a((high_pose.position.y - low_pose.position.y) / (high_pose.position.x - low_pose.position.x));
        const double b(low_pose.position.y - (a * low_pose.position.x));

        // x coordinate at point C (current_y_slice)
        const double xc((current_y_slice - b) / a);

        p.position.x = xc;
        p.position.z = current_y_slice; // Along Z axis
      }

      try
      {
        addLayer(msg, p, deposited_material_width, height_between_layers, number_of_passes,
                 towards_interior, slice_along_path, i % 2, angle_percentage, angle_type);
      }
      catch (std::runtime_error &e)
      {
        return e.what();
      }
      current_y_slice += height_between_layers;
      ++i;
    }
  }

  msg.poses.front().polygon_start = true;
  msg.poses.back().polygon_end = true;
  return "";
}

template<class ActionSpec>
void Profile<ActionSpec>::addLayer(ram_msgs::AdditiveManufacturingTrajectory &msg,
                                   geometry_msgs::Pose p,
                                   const double deposited_material_width,
                                   const double height_between_layers,
                                   const unsigned number_of_passes,
                                   const bool towards_interior,
                                   const bool slice_along_path,
                                   const bool invert,
                                   const double angle_percentage,
                                   const unsigned angle_type)
{
  unsigned layer_level(0);
  if (!msg.poses.empty())
    layer_level = msg.poses.back().layer_level + 1;

  double orientation_angle_interior(0);
  // Compute orientation angle on the interior
  // 0° == Z axis
  double delta_x(0);
  if (layer_level != 0)
  {
    if (!towards_interior)
    {
      double prev_layer_min_x(std::numeric_limits<double>::max());
      for (auto ram_pose : msg.poses)
      {
        if (ram_pose.layer_level < layer_level - 1)
          continue;
        if (ram_pose.layer_level > layer_level - 1)
          break;

        if (ram_pose.pose.position.x < prev_layer_min_x)
          prev_layer_min_x = ram_pose.pose.position.x;
      }
      delta_x = p.position.x - prev_layer_min_x;
    }
    else
    {
      double prev_layer_max_x(std::numeric_limits<double>::min());
      for (auto ram_pose : msg.poses)
      {
        if (ram_pose.layer_level < layer_level - 1)
          continue;
        if (ram_pose.layer_level > layer_level - 1)
          break;

        if (ram_pose.pose.position.x > prev_layer_max_x)
          prev_layer_max_x = ram_pose.pose.position.x;
      }
      delta_x = p.position.x - prev_layer_max_x;
    }

    if (delta_x == 0)
    {
      orientation_angle_interior = 0;
    }
    else
    {
      if (slice_along_path)
      {
        if (delta_x > 0)
          orientation_angle_interior = acos(delta_x / height_between_layers) - (M_PI / 2);
        else
          orientation_angle_interior = (M_PI / 2) - acos(delta_x / height_between_layers);
      }
      else
      {
        if (delta_x > 0)
          orientation_angle_interior = -((M_PI / 2) - atan(height_between_layers / delta_x));
        else
          orientation_angle_interior = atan(delta_x / height_between_layers);
      }
    }
  }
  orientation_angle_interior *= angle_percentage;

  std::vector<double> rings_x_coordinates;
  rings_x_coordinates.emplace_back(p.position.x);

  for (std::size_t i(1); i < number_of_passes; ++i)
  {
    double offset(i * deposited_material_width);
    if (towards_interior)
      offset = -offset;

    rings_x_coordinates.emplace_back(p.position.x + offset);
  }

  for (auto elem : rings_x_coordinates)
    if (elem < 0)
      throw std::runtime_error("A ring cannot be generated below zero, lower number of passes.");

  std::vector<double> ring_angles(rings_x_coordinates);
  for (std::size_t i(0); i < ring_angles.size(); ++i)
  {
    switch (angle_type)
    {
      case 0:
      {
        if (delta_x < 0)
          ring_angles.at(i) = orientation_angle_interior;
        else
          ring_angles.at(i) = -orientation_angle_interior;
        break;
      }
      case 1:
      {
        ring_angles.at(i) = (- 2 * orientation_angle_interior / (ring_angles.size() - 1)) * i + orientation_angle_interior;
        break;
      }
      case 2:
      {
        ring_angles.at(i) = (- orientation_angle_interior / (ring_angles.size() - 1)) * i;
        if (delta_x < 0)
          ring_angles.at(i) += orientation_angle_interior;
        break;
      }
      default:
        throw std::runtime_error("Angle type is not recognized");
        break;
    }
  }

  if (invert)
  {
    std::reverse(std::begin(rings_x_coordinates), std::end(rings_x_coordinates));
    std::reverse(std::begin(ring_angles), std::end(ring_angles));
  }

  if (towards_interior)
    std::reverse(std::begin(ring_angles), std::end(ring_angles));

  for (std::size_t i(0); i < rings_x_coordinates.size(); ++i)
    addRing(msg, rings_x_coordinates.at(i), p.position.z, layer_level, ring_angles.at(i));
}

template<class ActionSpec>
void Profile<ActionSpec>::addRing(ram_msgs::AdditiveManufacturingTrajectory &msg,
                                  const double radius,
                                  const double z_height,
                                  const unsigned layer_level,
                                  const double angle)
{
  ram_msgs::AdditiveManufacturingPose ram_pose;
  ram_pose.layer_level = layer_level;
  ram_pose.layer_index = ram_pose.layer_level;

  Eigen::Quaterniond q(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()));
  ram_pose.pose.orientation.x = q.x();
  ram_pose.pose.orientation.y = q.y();
  ram_pose.pose.orientation.z = q.z();
  ram_pose.pose.orientation.w = q.w();

  ram_pose.pose.position.z = z_height;

  ram_pose.pose.position.x = radius;
  msg.poses.emplace_back(ram_pose);
}

}

#endif
