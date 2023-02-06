#include <ram_post_processor/csv_post_processor.hpp>

namespace ram_post_processor
{

template<class charT>
struct no_separator : public std::numpunct_byname<charT>
{
  explicit no_separator(const char *name, size_t refs = 0)
      : std::numpunct_byname<charT>(name, refs)
  {
  }

protected:
  virtual std::string do_grouping() const
  {
    return "\000";  // groups of 0 (disable)
  }
};

CSVPostProcessor::CSVPostProcessor() : ram_post_processor::PostProcessor
                                           <ram_post_processor::GenerateCSVRequest, ram_post_processor::GenerateCSVResponse>
                                           ("CSV Post Processor",
                                            "A human readable textual output.\n"
                                            "Not meant to be used by any kind of hardware.",
                                            "ram/post_processors/csv")
{
}

void CSVPostProcessor::addPose(const bool,
                               const bool,
                               const bool,
                               const bool,
                               const bool,
                               const bool,
                               const ram_post_processor::GenerateCSVRequest &req,
                               ram_post_processor::GenerateCSVResponse &res)
{
  std::stringstream ss;

  try
  {
    if (req.separator_type == 0)
      ss.imbue(std::locale(std::locale(""), new no_separator<char>("fr_FR.utf8")));
    else
      ss.imbue(std::locale(std::locale(""), new no_separator<char>("C")));
  }
  catch (std::runtime_error &e)
  {
    res.error = "Could not change the locale for the numbers format!";
    return;
  }

  ss << std::fixed << std::setprecision(req.significant_digits);

  if (req.position_unit == 0) // Position in millimeter
  {
    ss << (currentPose()->pose.position.x) * 1000 << "; "
       << (currentPose()->pose.position.y) * 1000 << "; "
       << (currentPose()->pose.position.z) * 1000 << "; ";
  }
  else // Position in meter
  {
    ss << currentPose()->pose.position.x << "; "
       << currentPose()->pose.position.y << "; "
       << currentPose()->pose.position.z << "; ";
  }

  switch (req.orientation_convention)
  {
    default:
      throw std::runtime_error("Unknown orientation convention!");
      break;

    case 0: // Quaternions as orientation convention
      ss << currentPose()->pose.orientation.x << "; "
         << currentPose()->pose.orientation.y << "; "
         << currentPose()->pose.orientation.z << "; "
         << currentPose()->pose.orientation.w;
      break;

    case 1: // Fanuc yaW Pitch Roll as orientation convention
      Eigen::Quaterniond d;
      d.x() = currentPose()->pose.orientation.x;
      d.y() = currentPose()->pose.orientation.y;
      d.z() = currentPose()->pose.orientation.z;
      d.w() = currentPose()->pose.orientation.w;

      Eigen::Vector3d fanuc_yaw_pitch_roll(industrial_robot_angle_conversions::quaternionToFanucWPR(d));

      if (req.orientation_unit == 0) // Orientation in degrees
      {
        ss << fanuc_yaw_pitch_roll[0] * industrial_robot_angle_conversions::rad_2_deg << "; "
           << fanuc_yaw_pitch_roll[1] * industrial_robot_angle_conversions::rad_2_deg << "; "
           << fanuc_yaw_pitch_roll[2] * industrial_robot_angle_conversions::rad_2_deg;
      }
      else // Orientation in radians
      {
        ss << fanuc_yaw_pitch_roll[0] << "; "
           << fanuc_yaw_pitch_roll[1] << "; "
           << fanuc_yaw_pitch_roll[2];
      }
      break;
  }

  std::stringstream output_buffer;
  output_buffer << pose_number_++ << ";" << " " << ss.str();
  addComment(output_buffer.str(), res);
}

void CSVPostProcessor::addInformation(const ram_post_processor::GenerateCSVRequest &req,
                                      ram_post_processor::GenerateCSVResponse &res)
{
  PostProcessor::addInformation(req, res);

  std::string position_unit, orientation;
  if (req.position_unit == 0) // Position in millimeter
    position_unit = "millimeters";
  else // in meter
    position_unit = "meters";

  if (req.orientation_convention == 0) // orientation in quaternion convention
    orientation = "quaternion_x; quaternion_y; quaternion_z; quaternion_w";
  else // orientation in Fanuc convention
  {
    if (req.orientation_unit == 0) // Orientation in degrees
      orientation = "yaw_fanuc_degrees; pitch_fanuc_degrees; roll_fanuc_degrees";
    else // Orientation in radians
      orientation = "yaw_fanuc_radians; pitch_fanuc_radians; roll_fanuc_radians";
  }

  addComment(
    "pose_number; x_position_" + position_unit +
    "; y_position_" + position_unit + "; z_position_" + position_unit + "; " + orientation, res);
}

void CSVPostProcessor::afterGenerating(const ram_post_processor::GenerateCSVRequest &req,
                                       ram_post_processor::GenerateCSVResponse &res)
{
  if (req.information.record.save)
    saveToFiles(req.information.record.directory, ".csv", res);
}

void CSVPostProcessor::layerIndexChanged(const ram_post_processor::GenerateCSVRequest &,
                                         ram_post_processor::GenerateCSVResponse &)
{
  // Do nothing
}

void CSVPostProcessor::startFeedRateBefore(const ram_post_processor::GenerateCSVRequest &,
                                           ram_post_processor::GenerateCSVResponse &)
{
  // Do nothing
}

void CSVPostProcessor::startPolygonBefore(const ram_post_processor::GenerateCSVRequest &,
                                          ram_post_processor::GenerateCSVResponse &)
{
  // Do nothing
}

void CSVPostProcessor::finishPolygonAfter(const ram_post_processor::GenerateCSVRequest &,
                                          ram_post_processor::GenerateCSVResponse &)
{
  // Do nothing
}

void CSVPostProcessor::changeFeedRateBefore(const ram_post_processor::GenerateCSVRequest &,
                                            ram_post_processor::GenerateCSVResponse &)
{
  // Do nothing
}

void CSVPostProcessor::stopFeedRateBefore(const ram_post_processor::GenerateCSVRequest &,
                                          ram_post_processor::GenerateCSVResponse &)
{
  // Do nothing
}

void CSVPostProcessor::stopFeedRateAfter(const ram_post_processor::GenerateCSVRequest &,
                                         ram_post_processor::GenerateCSVResponse &)
{
  // Do nothing
}

void CSVPostProcessor::startLaserPowerBefore(const ram_post_processor::GenerateCSVRequest &,
                                             ram_post_processor::GenerateCSVResponse &)
{
  // Do nothing
}

void CSVPostProcessor::changeLaserPowerBefore(const ram_post_processor::GenerateCSVRequest &,
                                              ram_post_processor::GenerateCSVResponse &)
{
  // Do nothing
}

void CSVPostProcessor::stopLaserPowerBefore(const ram_post_processor::GenerateCSVRequest &,
                                            ram_post_processor::GenerateCSVResponse &)
{
  // Do nothing
}

void CSVPostProcessor::beforeGenerating(const ram_post_processor::GenerateCSVRequest &req,
                                        ram_post_processor::GenerateCSVResponse &res)
{
  if (req.start_at_0)
    pose_number_ = 0;
  else
    pose_number_ = 1;

  PostProcessor::beforeGenerating(req, res);
}

}
