#ifndef RAM_PATH_PLANNING_POLYGON_OFFSETS_IMP_HPP
#define RAM_PATH_PLANNING_POLYGON_OFFSETS_IMP_HPP

namespace ram_path_planning
{
using RamPosesIt = std::vector<ram_msgs::AdditiveManufacturingPose>::iterator;

template<class ActionSpec>
PolygonOffsets<ActionSpec>::PolygonOffsets() :
  PathPlanningAlgorithm<ActionSpec>("Polygon offsets", "Generate trajectories by offsetting an original trajectory",
                                    "ram/path_planning/generate_trajectory/polygon_offsets")
{
}

template<class ActionSpec>
bool PolygonOffsets<ActionSpec>::isPathClosed(ram_msgs::AdditiveManufacturingTrajectory &msg)
{
  bool closed_path(false);
  std::map<std::size_t, geometry_msgs::Point> indexed_points;

  for (auto &current_pose : msg.poses)
  {
    std::size_t i = &current_pose - &msg.poses.at(0);
    indexed_points.insert({i, current_pose.pose.position});
  }

  for (auto it_line(indexed_points.begin()); it_line != indexed_points.end(); ++it_line)
  {
    for (auto it(indexed_points.begin()); it != indexed_points.end(); ++it)
    {
      if (it == it_line)
        continue;
      if ((it->second.x == it_line->second.x) && (it->second.y == it_line->second.y) &&
          (it->second.z == it_line->second.z))
      {
        closed_path = true;
        break;
      }
    }
  }
  return closed_path;
}

template<class ActionSpec>
std::string PolygonOffsets<ActionSpec>::generateTrajectory(actionlib::ServerGoalHandle<ActionSpec>,
    ram_msgs::AdditiveManufacturingTrajectory &msg,
    const std::string file,
    const unsigned number_of_layers,
    const double height_between_layers,
    const double deposited_material_width,
    const double connection_value,
    const bool connection_type,
    const double offset_factor,
    const double safe_distance,
    const bool towards_interior,
    const unsigned number_of_passes,
    const unsigned join_type,
    const unsigned end_type,
    const double arc_tolerance,
    const double miter_limit,
    const unsigned arc_points,
    const bool discontinous_trajectory,
    const bool avoid_trajectories_crossing,
    const bool automatic_reverse_path,
    const bool reverse_origin_path)
{
  /*
   Diagram to illustrate the code to compute the connection angle

A' x { Offset trajectory
   |\   A x { Origin trajectory / pass - 1
   | \    |
   |  \   |
   |   \  |
   |    \ |
   |     \|
H  x------x S { Exit point
          |
          |
H' x      |
   |      |
   |    B x { Origin trajectory / pass - 1
B' x { Offset trajectory

  Points explanations:
  - S    : First exit point from the origin trajectory
  - H    : The orthographic projection point
  - H'   : The second (and other) exit points

  Segments explanations:
  - HH'  : The safe distance
  - HS   : Deposited width

  Vectors explanations:
  - A'S  : The projected vector in order to determine the orthographic projection point H
  - A'H  : The orthographic projection distance
  - A'B' : The base vector in order to compute the orthographic projection
  */

  msg.poses.clear();
  msg.file = file;
  msg.similar_layers = false;

  if (!strcasecmp(ram_utils::fileExtension(file).c_str(), "svg"))
  {
    // Read trajectory from SVG file
    ros::ServiceClient client = nh_.serviceClient<ram_utils::ParseSvgFile>("ram_utils/parse_svg_file");
    ram_utils::ParseSvgFile srv;
    srv.request.file_name = file;
    srv.request.arc_points = arc_points;

    if (!client.call(srv))
      return "Could not call service ram_utils or parse_svg_file";

    if (!srv.response.error.empty())
      return srv.response.error;

    if (srv.response.number_of_paths != 1)
      return "Only one path is allowed in the SVG file.";

    if (srv.response.traj.poses.size() < 3)
      return "Trajectory is too small, at least 2 segments are required in the drawing.";

    msg = srv.response.traj; // output : a complete contour of the given shape
  }
  else // Read trajectory from YAML File
  {
    if (!ram_utils::yamlFileToTrajectory(file, msg))
      return "Unknown error with YAML file";

    unsigned polygon_start_quantity(0);
    unsigned polygon_stop_quantity(0);
    for (auto current_pose : msg.poses)
    {
      if (current_pose.polygon_start)
        ++polygon_start_quantity;
      if (current_pose.polygon_end)
        ++polygon_stop_quantity;
    }
    if (polygon_start_quantity > 1 || polygon_stop_quantity > 1)
      return "Error with YAML file : the shape contains more than one polygon !";
  }
  // Disable Exit and Entry poses
  msg.poses.front().polygon_start = false;
  msg.poses.front().polygon_end = false;
  msg.poses.back().polygon_start = false;
  msg.poses.back().polygon_end = false;

  msg.generated = ros::Time::now();
  msg.modified = msg.generated;

  ram_msgs::AdditiveManufacturingTrajectory origin_trajectory{};
  std::vector<ram_msgs::AdditiveManufacturingTrajectory> offset_trajectories{};

  ClipperLib::Path orig;

  for (auto pose : msg.poses)
  {
    orig << ClipperLib::IntPoint(pose.pose.position.x * 1e6, pose.pose.position.y * 1e6);
    origin_trajectory.poses.emplace_back(pose);
  }

  msg.poses.clear(); // clean the gathered trajectory from YML or SVG in order to insert cleanly the computed trajectory

  ClipperLib::ClipperOffset co;
  auto join_t(static_cast<ClipperLib::JoinType>(join_type));
  auto end_t(static_cast<ClipperLib::EndType>(end_type));

  co.AddPath(orig, join_t, end_t);
  co.ArcTolerance = arc_tolerance;
  co.MiterLimit = miter_limit;

  bool is_path_closed = isPathClosed(origin_trajectory);

  if (!automatic_reverse_path && reverse_origin_path)
    std::reverse(origin_trajectory.poses.begin(), origin_trajectory.poses.end());

  for (std::size_t i(1); i < number_of_passes; ++i)
  {
    ram_msgs::AdditiveManufacturingTrajectory offset_trajectory;

    ClipperLib::Paths clipper_path;
    double offset(i * deposited_material_width * 1e6);
    if (towards_interior)
      offset = -offset;

    co.Execute(clipper_path, offset);
    if (clipper_path.empty())
      return "Offset path " + std::to_string(i) + " is empty";

    for (auto &path : clipper_path)
    {
      ram_msgs::AdditiveManufacturingPose last_pose;

      for (auto it_point(path.end() - 1); it_point != path.begin() - 1; --it_point)
      {
        ram_msgs::AdditiveManufacturingPose ram_pose;

        ram_pose.pose.orientation.x = ram_pose.pose.orientation.y = ram_pose.pose.orientation.z = 0;
        ram_pose.pose.orientation.w = 1;
        ram_pose.pose.position.z = 0;

        ram_pose.pose.position.x = (double) it_point->X / 1e6;
        ram_pose.pose.position.y = (double) it_point->Y / 1e6;
        offset_trajectory.poses.emplace_back(ram_pose);

        if (is_path_closed)
          if (it_point == path.end() - 1)
            last_pose = ram_pose; // To return at the first position for the closed path only
      }

      if (is_path_closed)
        offset_trajectory.poses.emplace_back(last_pose); // To return at the first position for the closed path only
    }
    offset_trajectories.emplace_back(offset_trajectory);
  }
  if (offset_trajectories.empty() && number_of_passes > 1)
    return "Error : Offset trajectory is empty after offsetting process";

  geometry_msgs::Point offset_start;
  // Gather start and end point in order to manage the entry / exit pose (open paths)
  if (!offset_trajectories.empty() && number_of_passes > 1)
    offset_start = offset_trajectories.front().poses.front().pose.position;

  if (automatic_reverse_path && number_of_passes > 1)
  {
    // Automatically invert the original path if the direction is opposite of the offset paths
    // First compute coordinates of the first vector of the original path
    Eigen::Vector3d origin_vector;
    std::size_t position_in_origin_vector(0);
    geometry_msgs::Point point_a(origin_trajectory.poses.at(position_in_origin_vector).pose.position);
    ++position_in_origin_vector; // Increment the position vector
    geometry_msgs::Point point_b(origin_trajectory.poses.at(position_in_origin_vector).pose.position);

    origin_vector.x() = point_b.x - point_a.x;
    origin_vector.y() = point_b.y - point_a.y;

    // Search for the closest points to the previous path
    Eigen::Vector3d offset_vector;
    geometry_msgs::Point point_c;
    geometry_msgs::Point point_d;
    std::multimap<double, geometry_msgs::Point> distance_map_point_c;
    std::map<double, geometry_msgs::Point> distance_map_point_d;
    auto offset_trajectory(offset_trajectories.front());
    double min_sqr_distance_a(std::nan("1"));
    double min_sqr_distance_b(std::nan("1"));
    int try_to_get_dot_product(3); // Try number before to cancel and stop the auto rotation process
    double dot_prod;
    int trial;

    // Search for the closest points to the previous path
    for (trial = 0; trial < try_to_get_dot_product; ++trial)
    {
      distance_map_point_c.clear();
      distance_map_point_d.clear();

      // First determine the D point, i.e. the closest point from B
      for (auto &pose : offset_trajectory.poses)
      {
        const double current_sqr_distance_a = std::pow(pose.pose.position.x - point_a.x, 2) +
                                              std::pow(pose.pose.position.y - point_a.y, 2);

        const double current_sqr_distance_b = std::pow(pose.pose.position.x - point_b.x, 2) +
                                              std::pow(pose.pose.position.y - point_b.y, 2);

        if ((std::isnan(min_sqr_distance_a) || current_sqr_distance_a <= min_sqr_distance_a))
        {
          min_sqr_distance_a = current_sqr_distance_a;
          distance_map_point_c.insert({min_sqr_distance_a, pose.pose.position});
        }

        if ((std::isnan(min_sqr_distance_b) || current_sqr_distance_b <= min_sqr_distance_b))
        {
          min_sqr_distance_b = current_sqr_distance_b;
          distance_map_point_d.insert({min_sqr_distance_b, pose.pose.position});
        }
      }
      point_d = distance_map_point_d.begin()->second;

      // Then find the C point, i.e. the closest point to A and D
      double min_sqr_distance_c_d(std::nan("1"));
      for (auto &line : distance_map_point_c)
      {
        const double current_sqr_distance_c_d = std::pow(line.second.x - point_d.x, 2) +
                                                std::pow(line.second.y - point_d.y, 2);

        if ((std::isnan(min_sqr_distance_c_d)) || (current_sqr_distance_c_d < min_sqr_distance_c_d))
        {
          min_sqr_distance_c_d = current_sqr_distance_c_d;

          if (!((line.second.x == point_d.x) && (line.second.y == point_d.y)))
            point_c = line.second;
        }
      }

      offset_vector.x() = point_d.x - point_c.x;
      offset_vector.y() = point_d.y - point_c.y;

      dot_prod = origin_vector.dot(offset_vector);
      if (!static_cast<bool>(dot_prod)) // In order to "filter" dot product null, because we can not conclude in this case
      {
        // Move a and b
        point_a = point_b; // Point A becomes B
        ++position_in_origin_vector; // The Point B is advanced
        point_b = origin_trajectory.poses.at(position_in_origin_vector).pose.position;
      }
      else
        break;
    }
    if (trial + 1 == try_to_get_dot_product)
      return "Auto rotation trajectory error"; // If the algorithm can not find a good geometric configuration

    // Study dot product signs ;
    // if dot product > 0 so the direction is equal, else the direction is opposed.
    bool same_direction(true);
    (dot_prod < 0) ? (same_direction = false) : (same_direction = true);
    if (!same_direction)
      std::reverse(origin_trajectory.poses.begin(), origin_trajectory.poses.end());
  }

  // Gather start and end point in order to manage the entry / exit pose (open paths)
  geometry_msgs::Point origin_end(origin_trajectory.poses.back().pose.position);

  if (number_of_passes > 1)
  {
    // This section is able to compute the connection angle
    if (is_path_closed)
    {
      // First compute the first exit pose within the origin trajectory (S point)
      ram_msgs::AdditiveManufacturingPose s_pose;
      Eigen::Vector3d ab_vector;
      // First remove the last pose the s_pose will be the new last pose
      origin_trajectory.poses.erase(origin_trajectory.poses.end());
      // Compute the norm the last section of the trajectory
      auto last(origin_trajectory.poses.begin());
      auto it(origin_trajectory.poses.end());
      auto second_to_last(std::prev(it, 1));
      ab_vector.x() = last->pose.position.x - second_to_last->pose.position.x;
      ab_vector.y() = last->pose.position.y - second_to_last->pose.position.y;
      double norm(ab_vector.norm());

      // Then compute the orientation of this vector
      double safe_factor(-safe_distance / norm);
      if (safe_distance >= norm)
        return "The safe distance is greater than the side of the shape for the origin trajectory";

      // Then find new coordinates
      s_pose.pose.position.x = origin_trajectory.poses.front().pose.position.x + safe_factor * ab_vector.x();
      s_pose.pose.position.y = origin_trajectory.poses.front().pose.position.y + safe_factor * ab_vector.y();

      // Add the new last exit point
      origin_trajectory.poses.emplace_back(s_pose);

      // For the next passes, the offset trajectory : compute the H and H' points
      int pass_number_index(0);
      bool switch_to_next_segment(false);
      for (auto it_traj(offset_trajectories.begin()); it_traj != offset_trajectories.end(); ++it_traj)
      {
        auto index = std::distance(offset_trajectories.begin(), it_traj);

        // Remove the last pose
        it_traj->poses.pop_back();

        // Find the closest previous exit pose
        double min_sqr_distance(std::nan("1"));
        double current_sqr_distance(0);
        auto it_min(it_traj->poses.begin());
        for (auto it(it_traj->poses.begin()); it != it_traj->poses.end(); ++it)
        {
          if (it_traj == offset_trajectories.begin())
            current_sqr_distance =
            std::pow(it->pose.position.x - origin_trajectory.poses.back().pose.position.x, 2) +
            std::pow(it->pose.position.y - origin_trajectory.poses.back().pose.position.y, 2);
          else
          {
            auto previous(std::prev(it_traj, 1));
            auto last_in_trajectory(previous->poses.begin());
            auto previous_first_pose(std::next(last_in_trajectory, 1));
            current_sqr_distance =
            std::pow(it->pose.position.x - previous_first_pose->pose.position.x, 2) +
            std::pow(it->pose.position.y - previous_first_pose->pose.position.y, 2);
          }
          if (std::isnan(min_sqr_distance) || current_sqr_distance < min_sqr_distance)
          {
            it_min = it;
            min_sqr_distance = current_sqr_distance;;
          }
        }
        // Auto rotation of the offset trajectory in order to begin with the new start point, the H point
        if (!std::isnan(min_sqr_distance))
        {
          last = it_min;
          std::rotate(it_traj->poses.begin(), it_min, it_traj->poses.end());
        }
        else
          return "Trajectory error with others passes version management";

        // Connection angle computation process
        auto first(it_traj->poses.begin());
        auto it(it_traj->poses.end());
        auto last(std::prev(it, 1));

        if (switch_to_next_segment)
        {
          it = it_traj->poses.end();
          first = std::prev(it, 1);
          last = std::prev(it, 2);
        }

        // 1
        // Compute the orientation vector
        Eigen::Vector3d b_prime_a_prime_vector;
        Eigen::Vector3d a_prime_s_vector;
        b_prime_a_prime_vector.x() = first->pose.position.x - last->pose.position.x;
        b_prime_a_prime_vector.y() = first->pose.position.y - last->pose.position.y;

        // Compute the norm of this vector
        double norm_b_prime_a_prime_vector(b_prime_a_prime_vector.norm());

        safe_factor = -safe_distance / norm_b_prime_a_prime_vector;
        if (safe_distance >= norm_b_prime_a_prime_vector)
          return "The safe distance is greater than the side of the shape for the pass number " + std::to_string(index);

        // Compute the dot product between the orientation vector and
        // the vector created by the previous S and the A' point
        // Only for the first element of offset_trajectories, it is compared to the original trajectory
        if (it_traj == offset_trajectories.begin())
        {
          a_prime_s_vector.x() = origin_trajectory.poses.back().pose.position.x - first->pose.position.x;
          a_prime_s_vector.y() = origin_trajectory.poses.back().pose.position.y - first->pose.position.y;
        }
        else if (switch_to_next_segment)
        {
          auto previous(std::prev(it_traj, 1));
          auto last_in_trajectory(previous->poses.end());
          auto second_to_last(std::prev(last_in_trajectory, 1));

          a_prime_s_vector.x() = second_to_last->pose.position.x - first->pose.position.x;
          a_prime_s_vector.y() = second_to_last->pose.position.y - first->pose.position.y;
          pass_number_index++;
        }
        else
        {
          auto previous_pass(offset_trajectories.begin());
          auto previous = std::next(previous_pass, pass_number_index);
          a_prime_s_vector.x() = previous->poses.back().pose.position.x - first->pose.position.x;
          a_prime_s_vector.y() = previous->poses.back().pose.position.y - first->pose.position.y;
          pass_number_index++;
        }

        // 2
        // Compute the distance between the exit point orthographic projection and the A' point
        // Compute the entry point (H) coordinates
        double sa_dot_orientation(a_prime_s_vector.dot(b_prime_a_prime_vector));
        // Compute the distance between the exit point orthographic projection and the A' point
        sa_dot_orientation /= std::pow(norm_b_prime_a_prime_vector, 2);

        ram_msgs::AdditiveManufacturingPose h_pose;
        // Temporary computation, if the connection distance or the angle are not changed, simple orthographic projection
        h_pose.pose.position.x = first->pose.position.x + sa_dot_orientation * b_prime_a_prime_vector.x();
        h_pose.pose.position.y = first->pose.position.y + sa_dot_orientation * b_prime_a_prime_vector.y();

        if (connection_type && connection_value != 0.0) //Connection distance management
        {
          double proportional_factor(connection_value / norm_b_prime_a_prime_vector);
          h_pose.pose.position.x += proportional_factor * b_prime_a_prime_vector.x();
          h_pose.pose.position.y += proportional_factor * b_prime_a_prime_vector.y();
        }
        else if (!connection_type && connection_value != M_PI_2) //Connection angle management
        {
          double connection_distance_temp = tan(M_PI_2 - connection_value) * deposited_material_width;
          double proportional_factor(connection_distance_temp / norm_b_prime_a_prime_vector);
          h_pose.pose.position.x -= proportional_factor * b_prime_a_prime_vector.x();
          h_pose.pose.position.y -= proportional_factor * b_prime_a_prime_vector.y();
        }

        Eigen::Vector3d a_prime_h;
        a_prime_h.x() = h_pose.pose.position.x - first->pose.position.x;
        a_prime_h.y() = h_pose.pose.position.y - first->pose.position.y;
        double a_prime_h_dot_b_prime_a_prime_vector(a_prime_h.dot(b_prime_a_prime_vector));

        if (a_prime_h_dot_b_prime_a_prime_vector >= 0)
          return "The pass number " + std::to_string(index + 2) + " does not respect the original shape, "
                 "\nplease change the parameters (connection angle or distance, end type)";

        // Compute the exit point (H') coordinates
        ram_msgs::AdditiveManufacturingPose h_prime_pose;
        h_prime_pose.pose.position.x = h_pose.pose.position.x + safe_factor * b_prime_a_prime_vector.x();
        h_prime_pose.pose.position.y = h_pose.pose.position.y + safe_factor * b_prime_a_prime_vector.y();

        // Offset factor, allows to offset location between current pass and pass + 1 with a law (SQRT, EXP or Linear TODO)
        if (offset_factor != 0.0)
        {
          double offset_factor_tmp(offset_factor);
          offset_factor_tmp += index; // Linear TODO
          h_prime_pose.pose.position.x += offset_factor_tmp * b_prime_a_prime_vector.x();
          h_prime_pose.pose.position.y += offset_factor_tmp * b_prime_a_prime_vector.y();
        }

        Eigen::Vector3d b_prime_h_prime;
        b_prime_h_prime.x() = h_prime_pose.pose.position.x - last->pose.position.x;
        b_prime_h_prime.y() = h_prime_pose.pose.position.y - last->pose.position.y;
        double b_h_prime_dot_b_prime_a_prime_vector(b_prime_h_prime.dot(b_prime_a_prime_vector));

        if (b_h_prime_dot_b_prime_a_prime_vector <= 0)
          return "The pass number " + std::to_string(index + 2) + " does not respect the original shape, "
                 "\nplease change the parameters (connection angle or distance, end type)";
        // 3
        // insert H as the first pose and H' as the last pose
        it_traj->poses.insert(it_traj->poses.begin(), h_pose);
        if (switch_to_next_segment)
        {
          it_traj->poses.insert(it_traj->poses.begin() + 1, it_traj->poses.back());
          it_traj->poses.pop_back();
        }
        // H' is the same than the end pose if the norm H'End < safe distance
        // End <=> B'
        // distance computation and comparison
        auto last_pose(it_traj->poses.end());
        last_pose = std::prev(last_pose, 1);
        double distance_h_prime_end = std::sqrt(pow((h_prime_pose.pose.position.x - last_pose->pose.position.x), 2)
                                                +
                                                pow((h_prime_pose.pose.position.y - last_pose->pose.position.y), 2));

        if (std::abs(distance_h_prime_end) >= safe_distance)
        {
          it_traj->poses.emplace_back(h_prime_pose);
          switch_to_next_segment = false;
        }
        else
          switch_to_next_segment = true;

        origin_trajectory.poses.insert(origin_trajectory.poses.end(), it_traj->poses.begin(), it_traj->poses.end());
      }
    }
    else  // For open path ONLY, in order to add offset trajectory to the origin trajectory
      for (auto &offset_trajectorie : offset_trajectories)
        origin_trajectory.poses.insert(origin_trajectory.poses.end(), offset_trajectorie.poses.begin(),
                                       offset_trajectorie.poses.end());
  }

  // Start and End poses management for discontinous trajectories
  if (discontinous_trajectory)
  {
    origin_trajectory.poses.front().polygon_end = false;
    origin_trajectory.poses.front().polygon_start = true;
    origin_trajectory.poses.back().polygon_end = true;
    origin_trajectory.poses.back().polygon_start = false;
  }

  // Add layers in Z section
  for (std::size_t k(0); k < number_of_layers; ++k)
  {
    for (auto &pose : origin_trajectory.poses)
    {
      pose.pose.position.z = k * height_between_layers;
      pose.layer_level = pose.layer_index = k;
    }
    // Stacking process
    // For discontinous trajectories, only stack layers and repeat the same trajectory
    if (discontinous_trajectory)
      msg.poses.insert(msg.poses.end(), origin_trajectory.poses.begin(), origin_trajectory.poses.end());
    // Continous trajectories, 1 layer every 2 is inverted
    else
    {
      if (k % 2) // Odd layer are inverted
      {
        std::reverse(origin_trajectory.poses.begin(), origin_trajectory.poses.end()); // invert the poses
        msg.poses.insert(msg.poses.end(), origin_trajectory.poses.begin(),
                         origin_trajectory.poses.end()); // Insert the poses
        std::reverse(origin_trajectory.poses.begin(),
                     origin_trajectory.poses.end()); // reset the poses for the next layer
      }
      else // Peer layers
        msg.poses.insert(msg.poses.end(), origin_trajectory.poses.begin(), origin_trajectory.poses.end());
    }
  }

  // Entry / exit poses management for open path between the first pass and other passes
  if (!is_path_closed && avoid_trajectories_crossing)
  {
    for (auto &pose : msg.poses)
    {
      // For the last origin pose
      if ((pose.pose.position.x == origin_end.x && pose.pose.position.y == origin_end.y))
      {
        // If the trajectory is continous and if the layer level is odd
        if (!discontinous_trajectory && pose.layer_level % 2)
        {
          pose.polygon_start = true;
          pose.polygon_end = false;
        }
        else // Either the trajectory is discontinous or the layer level is peer
        {
          pose.polygon_start = false;
          pose.polygon_end = true;
        }
      }
      // For the first offset pose
      if ((pose.pose.position.x == offset_start.x && pose.pose.position.y == offset_start.y))
      {
        // If the trajectory is continous and if the layer level is odd
        if (!discontinous_trajectory && pose.layer_level % 2)
        {
          pose.polygon_start = false;
          pose.polygon_end = true;
        }
        else // Either the trajectory is discontinous or the layer level is peer
        {
          pose.polygon_start = true;
          pose.polygon_end = false;
        }
      }
    }
  }
  // First Start and Final End poses management
  if (!discontinous_trajectory)
  {
    msg.poses.front().polygon_start = true;
    msg.poses.front().polygon_end = false;
    msg.poses.back().polygon_start = false;
    msg.poses.back().polygon_end = true;
  }

  return "";
}
}
#endif
