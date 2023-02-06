#include <mutex>
#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_utils/BufferParams.h>
#include <ram_utils/ExportTrajectory.h>
#include <ram_utils/file_extension.hpp>
#include <ram_utils/ImportTrajectory.h>
#include <ram_utils/UnmodifiedTrajectory.h>
#include <rosbag/bag.h>
#include <rosbag/buffer.h>
#include <rosbag/exceptions.h>
#include <rosbag/structures.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <signal.h>
#include <string>
#include <strings.h>

rosbag::Bag buffer_bag;

const unsigned msg_to_keep(10);
unsigned last_msg_published = 0;
ros::Publisher pub;

std::mutex msgs_mutex;
std::vector<ram_msgs::AdditiveManufacturingTrajectory> traj_msgs; // Buffer 1. Contain all msgs

std::mutex unmodified_msgs_mutex;
std::vector<ram_msgs::AdditiveManufacturingTrajectory> unmodified_traj_msgs; // Buffer 2. Contain just the unmodified msgs

// On shutdown, write bag file containing last trajectories
void mySigintHandler(int)
{
  std::lock_guard<std::mutex> lock(msgs_mutex);
  std::lock_guard<std::mutex> lock_1(unmodified_msgs_mutex);

  if ((last_msg_published + 1) < traj_msgs.size()) // erase msgs
  {
    traj_msgs.erase((traj_msgs.begin() + last_msg_published + 1), traj_msgs.end());
    // Erase msgs in the second buffer
    while (unmodified_traj_msgs.back().generated != traj_msgs.back().generated && !unmodified_traj_msgs.empty())
      unmodified_traj_msgs.pop_back();
  }

  buffer_bag.open("ram_buffer_trajectory.bag", rosbag::bagmode::Write);
  for (auto msg : traj_msgs)
    buffer_bag.write("ram/trajectory", ros::Time::now(), msg);
  buffer_bag.close();

  buffer_bag.open("ram_buffer_unmodified_trajectory.bag", rosbag::bagmode::Write);
  for (auto msg : unmodified_traj_msgs)
    buffer_bag.write("ram/trajectory", ros::Time::now(), msg);
  buffer_bag.close();

  ros::shutdown();
}

void saveTrajectoryCallback(const ros::MessageEvent<ram_msgs::AdditiveManufacturingTrajectory> &msg_event)
{
  if (ros::this_node::getName().compare(msg_event.getPublisherName()) == 0) // The publisher is himself
    return;

  std::lock_guard<std::mutex> lock(msgs_mutex);
  std::lock_guard<std::mutex> lock_1(unmodified_msgs_mutex);

  if ((last_msg_published + 1) < traj_msgs.size()) // erase msgs
  {
    traj_msgs.erase((traj_msgs.begin() + last_msg_published + 1), traj_msgs.end());
    // Erase msgs in the second buffer
    while (unmodified_traj_msgs.back().generated != traj_msgs.back().generated && !unmodified_traj_msgs.empty())
      unmodified_traj_msgs.pop_back();
  }

  last_msg_published = traj_msgs.size();
  traj_msgs.push_back(*msg_event.getMessage());

  if (msg_event.getMessage()->generated == msg_event.getMessage()->modified) // the trajectory is not modified
    unmodified_traj_msgs.push_back(*msg_event.getMessage());
  // Always save the same numbers of elements
  if (traj_msgs.size() > msg_to_keep)
  {
    traj_msgs.erase(traj_msgs.begin(), traj_msgs.end() - msg_to_keep);
    last_msg_published = traj_msgs.size() - 1;
  }
  if (unmodified_traj_msgs.size() > msg_to_keep)
    unmodified_traj_msgs.erase(unmodified_traj_msgs.begin(), unmodified_traj_msgs.end() - msg_to_keep);
}

bool trajectoryBufferCallback(ram_utils::BufferParams::Request &req,
                              ram_utils::BufferParams::Response &res)
{
  std::lock_guard<std::mutex> lock(msgs_mutex);

  ram_msgs::AdditiveManufacturingTrajectory trajectory;
  switch (req.button_id)
  {
    case 1:
      // Back button
      if (last_msg_published > 0)
        --last_msg_published;
      else
      {
        res.error = "Start of the buffer";
        return true;
      }
      break;
    case 2:
      // Forward button
      if ((last_msg_published + 1) < traj_msgs.size())
        ++last_msg_published;
      else
      {
        res.error = "End of the buffer";
        return true;
      }
      break;
  }

  if (last_msg_published < traj_msgs.size())
  {
    // Trajectory is published
    trajectory = traj_msgs[last_msg_published];
    pub.publish(trajectory);
  }

  res.error.clear();
  return true;
}

bool getUnmodifiedTrajectoryCallback(ram_utils::UnmodifiedTrajectory::Request &req,
                                     ram_utils::UnmodifiedTrajectory::Response &res)
{
  std::lock_guard<std::mutex> lock_1(unmodified_msgs_mutex);

  for (auto msg : unmodified_traj_msgs)
  {
    if (msg.generated == req.generated)
    {
      res.trajectory = msg;
      return true;
    }
  }

  res.error = "Cannot find corresponding unmodified trajectory";
  return true;
}

bool exportTrajectoryCallback(ram_utils::ExportTrajectory::Request &req,
                              ram_utils::ExportTrajectory::Response &res)
{
  std::lock_guard<std::mutex> lock(msgs_mutex);
  std::lock_guard<std::mutex> lock_1(unmodified_msgs_mutex);

  if (req.file_name.find("smb-share:") != std::string::npos)
  {
    res.error = "Cannot export file to network!\n" + req.file_name;
    return true;
  }

  std::string file_extension(ram_utils::fileExtension(req.file_name));
  if (file_extension.compare("bag") != 0)
  {
    res.error = req.file_name + " is not a bag file";
    return true;
  }

  // Find current trajectory
  if (last_msg_published >= traj_msgs.size())
  {
    res.error = "Current trajectory is empty";
    return true;
  }

  ram_msgs::AdditiveManufacturingTrajectory current_traj = traj_msgs[last_msg_published];
  ram_msgs::AdditiveManufacturingTrajectory unmodified_current_traj = current_traj;

  if (current_traj.poses.size() == 0)
  {
    res.error = "Current trajectory is empty";
    return true;
  }

  // Find unmodified trajectory
  bool trajectory_in_buffer = false;
  for (auto msg : unmodified_traj_msgs)
  {
    if (current_traj.generated != msg.generated)
      continue;
    // Trajectories are equals
    trajectory_in_buffer = true;
    if (current_traj.modified != msg.modified)
    {
      unmodified_current_traj = msg;
      break;
    }
  }

  if (!trajectory_in_buffer)
  {
    res.error = "Corresponding unmodified trajectory does not exist in the buffer";
    return true;
  }

  // Save trajectory
  rosbag::Bag export_trajectoy_bag;
  try
  {
    export_trajectoy_bag.open(req.file_name, rosbag::bagmode::Write);
    export_trajectoy_bag.write("ram/trajectory", ros::Time::now(), current_traj);
    export_trajectoy_bag.write("ram/trajectory", ros::Time::now(), unmodified_current_traj);
    export_trajectoy_bag.close();
  }
  catch (rosbag::BagException &e)
  {
    export_trajectoy_bag.close();
    ROS_ERROR_STREAM(e.what());
    res.error = e.what();
    return true;
  }

  return true;
}

bool importTrajectoryCallback(ram_utils::ImportTrajectory::Request &req,
                              ram_utils::ImportTrajectory::Response &res)
{
  std::lock_guard<std::mutex> lock(msgs_mutex);
  std::lock_guard<std::mutex> lock_1(unmodified_msgs_mutex);

  std::string file_extension(ram_utils::fileExtension(req.file_name));
  if (file_extension.compare("bag") != 0)
  {
    res.error = req.file_name + " is not a bag file";
    return true;
  }

  rosbag::Bag import_trajectoy_bag;

  std::vector<ram_msgs::AdditiveManufacturingTrajectory> msgs_from_file;
  try
  {
    import_trajectoy_bag.open(req.file_name, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("ram/trajectory"));
    rosbag::View view(import_trajectoy_bag, rosbag::TopicQuery(topics));

    for (auto m : view)
    {
      ram_msgs::AdditiveManufacturingTrajectory::ConstPtr s =
          m.instantiate<ram_msgs::AdditiveManufacturingTrajectory>();
      if (s != NULL)
        msgs_from_file.push_back(*s);
    }

    import_trajectoy_bag.close();
  }
  catch (rosbag::BagException &e)
  {
    import_trajectoy_bag.close();
    ROS_ERROR_STREAM(e.what());
    res.error = e.what();
    return true;
  }

  // First message = trajectory
  // Second message = original trajectory
  ram_msgs::AdditiveManufacturingTrajectory unmodified_trajectory;
  if (msgs_from_file.empty())
  {
    res.error = "Unable to import trajectory";
    return true;
  }
  else if (msgs_from_file.size() == 1)
  {
    unmodified_trajectory = msgs_from_file[0];
  }
  else if (msgs_from_file.size() == 2)
  {
    unmodified_trajectory = msgs_from_file[1];
  }

  ram_msgs::AdditiveManufacturingTrajectory trajectory = msgs_from_file[0];
  if (trajectory.poses.size() == 0)
  {
    res.error = "Trajectory is empty";
    return true;
  }

  // Check for duplicate UUIDs
  std::map<uuid_msgs::UniqueID::_uuid_type, int> count_map;
  for (auto &p : trajectory.poses)
  {
    if (count_map.count(p.unique_id.uuid) == 0)
      count_map.emplace(p.unique_id.uuid, 1);
    else
      count_map.at(p.unique_id.uuid)++;
  }

  for (auto k: count_map)
  {
    if (k.second > 1)
    {
      res.error = "Duplicate UUIDs in the trajectory!";
      return true;
    }
  }

  // Save unmodified_trajectory
  for (unsigned i(0); i < unmodified_traj_msgs.size(); ++i)
  {
    if (unmodified_trajectory.generated == unmodified_traj_msgs[i].generated)
    {
      unmodified_traj_msgs.erase(unmodified_traj_msgs.begin() + i); // avoid duplicate duplicate in unmodified buffer
      break;
    }
  }
  // Save trajectories
  unmodified_traj_msgs.push_back(unmodified_trajectory);
  traj_msgs.push_back(trajectory);
  last_msg_published = traj_msgs.size() - 1;

  // Publish trajectory
  pub.publish(trajectory);
  return true;
}

int main(int argc,
         char **argv)
{
  ros::init(argc, argv, "ram_utils", ros::init_options::NoSigintHandler);
  signal(SIGINT, mySigintHandler);
  ros::NodeHandle nh;

  bool error_in_first_bag = false;
  bool error_in_second_bag = false;

  // Read the bag file (first buffer)
  try
  {
    buffer_bag.open("ram_buffer_trajectory.bag", rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("ram/trajectory"));
    rosbag::View view(buffer_bag, rosbag::TopicQuery(topics));
    std::lock_guard<std::mutex> lock(msgs_mutex);

    for (auto m : view)
    {
      ram_msgs::AdditiveManufacturingTrajectory::ConstPtr s =
          m.instantiate<ram_msgs::AdditiveManufacturingTrajectory>();
      if (s != NULL)
        traj_msgs.push_back(*s);
    }

    last_msg_published = traj_msgs.size(); // There are not published trajectories
    buffer_bag.close();
  }
  catch (rosbag::BagException &e)
  {
    buffer_bag.close();
    ROS_WARN_STREAM(e.what());

    std::string file(getenv("HOME"));
    file += "/.ros/ram_buffer_trajectory.bag";

    if (std::remove(file.c_str()) == 0)
      ROS_WARN_STREAM(file << " was removed because corrupted");

    error_in_first_bag = true;
  }

  // Read the bag file (second buffer)
  try
  {
    buffer_bag.open("ram_buffer_unmodified_trajectory.bag", rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("ram/trajectory"));
    rosbag::View view(buffer_bag, rosbag::TopicQuery(topics));
    std::lock_guard<std::mutex> lock_1(unmodified_msgs_mutex);

    for (auto m : view)
    {
      ram_msgs::AdditiveManufacturingTrajectory::ConstPtr s =
          m.instantiate<ram_msgs::AdditiveManufacturingTrajectory>();
      if (s != NULL)
        unmodified_traj_msgs.push_back(*s);
    }
    buffer_bag.close();
  }
  catch (rosbag::BagException &e)
  {
    buffer_bag.close();
    ROS_WARN_STREAM(e.what());

    std::string file(getenv("HOME"));
    file += "/.ros/ram_buffer_unmodified_trajectory.bag";

    if (std::remove(file.c_str()) == 0)
      ROS_WARN_STREAM(file << " was removed because corrupted");

    error_in_second_bag = true;
  }

  if (error_in_first_bag || error_in_second_bag)
  {
    traj_msgs.clear();
    unmodified_traj_msgs.clear();
    last_msg_published = 0;
  }
  // Publish on "ram/trajectory"
  pub = nh.advertise<ram_msgs::AdditiveManufacturingTrajectory>("ram/trajectory", msg_to_keep, true);

  // Subscribe on "ram/trajectory"
  ros::Subscriber sub = nh.subscribe("ram/trajectory", msg_to_keep, saveTrajectoryCallback);
  ros::ServiceServer service_1 = nh.advertiseService("ram/buffer/get_trajectory", trajectoryBufferCallback);
  ros::ServiceServer service_2 = nh.advertiseService("ram/buffer/get_unmodified_trajectory",
                                                     getUnmodifiedTrajectoryCallback);
  ros::ServiceServer service_3 = nh.advertiseService("ram/buffer/export_trajectory", exportTrajectoryCallback);
  ros::ServiceServer service_4 = nh.advertiseService("ram/buffer/import_trajectory", importTrajectoryCallback);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
