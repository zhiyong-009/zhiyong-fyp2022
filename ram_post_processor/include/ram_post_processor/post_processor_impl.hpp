#ifndef RAM_POST_PROCESSOR_POST_PROCESSOR_IMPL_HPP
#define RAM_POST_PROCESSOR_POST_PROCESSOR_IMPL_HPP

namespace ram_post_processor
{

template<class ServiceReq, class ServiceRes>
PostProcessor<ServiceReq, ServiceRes>::PostProcessor(const std::string name,
    const std::string description,
    const std::string service_name) :
  name_(name),
  description_(description),
  service_name_(service_name)
{
  get_start_pose_client_ = nh_.serviceClient<ram_utils::GetStartPose>("ram/get_start_pose");
  get_tool_client_ = nh_.serviceClient<ram_utils::GetTool>("ram/get_tool");
  get_trajectory_infos_client_ = nh_.serviceClient<ram_utils::GetTrajectoryInfos>(
                                 "ram/information/get_trajectory_infos");
}

template<class ServiceReq, class ServiceRes>
PostProcessor<ServiceReq, ServiceRes>::~PostProcessor()
{
}


/// Generic Methods
template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::addComment(const std::string &comment,
    ServiceRes &res)
{
  res.programs.front().program.append(comment + '\n');
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::addPose(const bool,
    const bool,
    const bool,
    const bool,
    const bool,
    const bool,
    const ServiceReq &,
    ServiceRes &)
{
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::clearProgram(const ServiceReq &, ServiceRes &res)
{
  res.programs.clear();
  res.error.clear();
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::beforeGenerating(const ServiceReq &req, ServiceRes &res)
{
  addInformation(req, res);
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::afterGenerating(const ServiceReq &req, ServiceRes &res)
{
  if (req.information.record.save)
    saveToFiles(req.information.record.directory, ".txt", res);
}

// Polygons
template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::startPolygonBefore(const ServiceReq &, ServiceRes &res)
{
  addComment("Start polygon", res);
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::startPolygonAfter(const ServiceReq &, ServiceRes &)
{
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::finishPolygonBefore(const ServiceReq &, ServiceRes &)
{
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::finishPolygonAfter(const ServiceReq &req, ServiceRes &res)
{
  if (req.information.verbose)
    addComment("Finish polygon", res);
}

// Layer change
template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::layerIndexChanged(const ServiceReq &req, ServiceRes &res)
{
  if (req.information.verbose)
  {
    addComment("Layer index " + std::to_string(currentPose()->layer_index), res);
    addComment("Layer level " + std::to_string(currentPose()->layer_level), res);
  }
}

// Feed rate
template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::startFeedRateBefore(const ServiceReq &, ServiceRes &res)
{
  addComment("Start feed rate: " + std::to_string(currentPose()->params.feed_rate), res);
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::startFeedRateAfter(const ServiceReq &, ServiceRes &)
{
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::changeFeedRateBefore(const ServiceReq &, ServiceRes &res)
{
  addComment("Change feed rate: " + std::to_string(currentPose()->params.feed_rate), res);
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::changeFeedRateAfter(const ServiceReq &, ServiceRes &)
{
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::stopFeedRateBefore(const ServiceReq &, ServiceRes &res)
{
  addComment("Stop feed rate", res);
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::stopFeedRateAfter(const ServiceReq &, ServiceRes &res)
{
  addComment("Stop feed rate", res);
}

// Laser power
template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::startLaserPowerBefore(const ServiceReq &, ServiceRes &res)
{
  addComment("Start laser_power: " + std::to_string(currentPose()->params.laser_power), res);
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::startLaserPowerAfter(const ServiceReq &, ServiceRes &)
{
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::changeLaserPowerBefore(const ServiceReq &, ServiceRes &res)
{
  addComment("Change laser power: " + std::to_string(currentPose()->params.laser_power), res);
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::changeLaserPowerAfter(const ServiceReq &, ServiceRes &)
{
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::stopLaserPowerBefore(const ServiceReq &, ServiceRes &res)
{
  addComment("Stop laser power", res);
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::stopLaserPowerAfter(const ServiceReq &, ServiceRes &)
{
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::checkTrajectory(ServiceRes &res, ram_msgs::AdditiveManufacturingTrajectory &trajectory)
{
  if (trajectory.poses.empty())
    res.error += "- Trajectory is empty, please generate it \n";
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::checkPoses(ram_msgs::AdditiveManufacturingTrajectory &trajectory)
{
  for (auto pose : trajectory.poses)
  {
    if (std::isnan(pose.pose.orientation.x))
      throw std::runtime_error("Pose orientation X coordinate is NaN");
    if (std::isnan(pose.pose.orientation.y))
      throw std::runtime_error("Pose orientation Y coordinate is NaN");
    if (std::isnan(pose.pose.orientation.z))
      throw std::runtime_error("Pose orientation Z coordinate is NaN");
    if (std::isnan(pose.pose.orientation.w))
      throw std::runtime_error("Pose orientation W coordinate is NaN");
    if (std::isnan(pose.pose.position.x))
      throw std::runtime_error("Pose position X coordinate is NaN");
    if (std::isnan(pose.pose.position.y))
      throw std::runtime_error("Pose position Y coordinate is NaN");
    if (std::isnan(pose.pose.position.z))
      throw std::runtime_error("Pose position Z coordinate is NaN");
  }

  trajectory_ = trajectory;
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::transformAllPoses(ram_msgs::AdditiveManufacturingTrajectory &trajectory, Eigen::Isometry3d start_pose,
    Eigen::Isometry3d tool)
{
  Eigen::Isometry3d pose;
  for (auto &ram_pose : trajectory.poses)
  {
    // geometry_msg to Eigen
    tf::poseMsgToEigen(ram_pose.pose, pose);
    // Transform pose
    pose = start_pose * pose * tool;
    // Eigen to geometry
    tf::poseEigenToMsg(pose, ram_pose.pose);
  }
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::startPoseService()
{
  // Check that service exists
  if (!get_start_pose_client_.waitForExistence(ros::Duration(0.5)))
    throw std::runtime_error("Cannot get start pose, service does not exist");

  // Call service to get start pose
  ram_utils::GetStartPose srv;
  if (!get_start_pose_client_.call(srv))
    throw std::runtime_error("Cannot get start pose, call to service failed");

  tf::poseMsgToEigen(srv.response.pose, start_pose_);
}

template<class ServiceReq, class ServiceRes>
bool
PostProcessor<ServiceReq, ServiceRes>::getTrajectoryInfo(double &seconds)
{
  // Check that service exists
  if (!get_trajectory_infos_client_.waitForExistence(ros::Duration(0.5)))
    throw std::runtime_error("Cannot get trajectory infos, service does not exist");

  ram_utils::GetTrajectoryInfos srv;
  // Call service to get traj infos
  if (!get_trajectory_infos_client_.call(srv))
    throw std::runtime_error("Cannot get trajectory infos, call to service failed");

  seconds = srv.response.informations.execution_time;
  return true;
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::callToolService()
{
  // Check that service exists
  if (!get_tool_client_.waitForExistence(ros::Duration(0.5)))
    throw std::runtime_error("Cannot get tool, service does not exist");

  // Call service to get start pose
  ram_utils::GetTool srv;
  if (!get_tool_client_.call(srv))
    throw std::runtime_error("Cannot get tool, call to service failed");

  tf::poseMsgToEigen(srv.response.pose, tool_);
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::checkParameters(const ServiceReq &req, ServiceRes &res)
{
  if (req.information.name.empty())
    res.error += "- Please specify the program name \n";

  if (req.information.record.save)
  {
    struct stat buffer {};
    if (req.information.record.directory.empty())
      res.error += "- Please specify the output directory for the program \n";
    else if (stat(req.information.record.directory.c_str(), &buffer) == -1)
      res.error += "- Specified directory does not exist \n";
  }
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::addProgram(const ServiceReq &, ServiceRes &res, const std::string &name)
{
  ram_post_processor::RobotProgram temp_robot_program;
  temp_robot_program.name = name;
  res.programs.emplace_back(temp_robot_program);
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::addInformation(const ServiceReq &req, ServiceRes &res)
{
  if (req.information.verbose)
  {
    addComment("File = " + trajectory_.file, res);
    addComment(trajectory_.generation_info, res);

    double seconds_temp;

    if (!getTrajectoryInfo(seconds_temp))
      throw std::runtime_error("Cannot get trajectory infos, call to service failed");

    uint16_t seconds(seconds_temp);
    unsigned hours(seconds / 3600);
    seconds -= hours * 3600;

    unsigned minutes(seconds / 60);
    seconds -= minutes * 60;

    addComment(
    "Execution duration estimated (hh:mm:ss): " + std::to_string(hours) + ":" + std::to_string(minutes) + ":"
    + std::to_string(seconds), res);
  }
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::processRequest(const ServiceReq &req, ServiceRes &res,
    ram_msgs::AdditiveManufacturingTrajectory &trajectory)
{
  checkTrajectory(res, trajectory);
  checkParameters(req, res);
  if (!res.error.empty())
    throw std::runtime_error(res.error);

  clearProgram(req, res);
  addProgram(req, res, req.information.name);
  checkPoses(trajectory);
  addComment("Program comment: " + req.information.comment, res);
  startPoseService();
  callToolService();
  current_pose_ = trajectory_.poses.begin();
  beforeGenerating(req, res);

  /// Transform all poses using start pose and tool orientation
  transformAllPoses(trajectory_, start_pose_, tool_);

  bool first_pose(true);
  ram_msgs::AdditiveManufacturingPose last_pose;

  for (current_pose_ = trajectory_.poses.begin(); current_pose_ != trajectory_.poses.end(); ++current_pose_)
  {
    bool laser_power_start(false),
         laser_power_stop(false),
         laser_power_change(false),
         feed_rate_start(false),
         feed_rate_stop(false),
         feed_rate_change(false);

    // (Layer index changed OR first pose)
    if ((current_pose_->layer_index != last_pose.layer_index || first_pose))
      layerIndexChanged(req, res);

    if (current_pose_->polygon_start)
      startPolygonBefore(req, res);

    if (current_pose_->polygon_end)
      finishPolygonBefore(req, res);

    if (first_pose)
    {
      if (current_pose_->params.feed_rate != 0)
      {
        feed_rate_start = true;
        startFeedRateBefore(req, res);
      }

      if (last_pose.params.laser_power != 0)
      {
        laser_power_start = true;
        startLaserPowerBefore(req, res);
      }
    }
    else
    {
      // Feed rate changes
      if (current_pose_->params.feed_rate != last_pose.params.feed_rate)
      {
        if (last_pose.params.feed_rate != 0 && current_pose_->params.feed_rate == 0)
        {
          feed_rate_stop = true;
          stopFeedRateBefore(req, res);
        }
        else if (last_pose.params.feed_rate == 0)
        {
          feed_rate_start = true;
          startFeedRateBefore(req, res);
        }
        else
        {
          feed_rate_change = true;
          changeFeedRateBefore(req, res);
        }
      }

      // Laser power changes
      if (current_pose_->params.laser_power != last_pose.params.laser_power)
      {
        if (last_pose.params.laser_power != 0 && current_pose_->params.laser_power == 0)
        {
          laser_power_stop = true;
          stopLaserPowerBefore(req, res);
        }
        else if (last_pose.params.laser_power == 0)
        {
          laser_power_start = true;
          startLaserPowerBefore(req, res);
        }
        else
        {
          laser_power_change = true;
          changeLaserPowerBefore(req, res);
        }
      }
    }
    /// Add transformed pose
    addPose(laser_power_start,
            laser_power_stop,
            laser_power_change,
            feed_rate_start,
            feed_rate_stop,
            feed_rate_change,
            req, res);

    /// Actions AFTER adding pose
    if (current_pose_->polygon_start)
      startPolygonAfter(req, res);

    if (current_pose_->polygon_end)
      finishPolygonAfter(req, res);

    if (first_pose)
    {
      if (current_pose_->params.feed_rate != 0)
        startFeedRateAfter(req, res);

      if (last_pose.params.laser_power != 0)
        startLaserPowerAfter(req, res);
    }
    else
    {
      // Feed rate changes
      if (current_pose_->params.feed_rate != last_pose.params.feed_rate)
      {
        if (last_pose.params.feed_rate != 0 && current_pose_->params.feed_rate == 0)
          stopFeedRateAfter(req, res);
        else if (last_pose.params.feed_rate == 0)
          startFeedRateAfter(req, res);
        else
          changeFeedRateAfter(req, res);
      }

      // Laser power changes
      if (current_pose_->params.laser_power != last_pose.params.laser_power)
      {
        if (last_pose.params.laser_power != 0 && current_pose_->params.laser_power == 0)
          stopLaserPowerAfter(req, res);
        else if (last_pose.params.laser_power == 0)
          startLaserPowerAfter(req, res);
        else
          changeLaserPowerAfter(req, res);
      }
    }

    first_pose = false;
    last_pose = *current_pose_;
  }

  afterGenerating(req, res);
}

template<class ServiceReq, class ServiceRes>
void
PostProcessor<ServiceReq, ServiceRes>::saveToFiles(const std::string &directory,
    const std::string &file_extension, ServiceRes &res)
{
  std::ofstream program_file;
  for (auto &program : res.programs)
  {
    const std::string file_name(directory + "/" + program.name + file_extension);
    program_file.open(file_name, std::ofstream::out);

    if (!program_file.is_open())
      throw std::runtime_error("Could not open file " + file_name + " for writing");

    program_file << program.program;
    program_file.close();
  }
}

template<class ServiceReq, class ServiceRes>
const ram_msgs::AdditiveManufacturingTrajectory &
PostProcessor<ServiceReq, ServiceRes>::traj()
{
  return trajectory_;
}

template<class ServiceReq, class ServiceRes>
const ram_pose_const_iterator &
PostProcessor<ServiceReq, ServiceRes>::currentPose()
{
  return current_pose_;
}

}

#endif
