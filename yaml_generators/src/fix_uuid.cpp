#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_utils/file_extension.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <unique_id/unique_id.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "fix_uuid");
  ros::NodeHandle nh("~");

  std::string file_name;
  if (!nh.getParam("file", file_name))
  {
    ROS_ERROR_STREAM("file parameter must be provided");
    return 1;
  }

  std::string file_extension(ram_utils::fileExtension(file_name));
  if (file_extension.compare("bag") != 0)
  {
    ROS_ERROR_STREAM("file parameter must be a bag file");
    return 1;
  }

  rosbag::Bag import_trajectoy_bag;
  std::vector<ram_msgs::AdditiveManufacturingTrajectory> msgs_from_file;
  try
  {
    import_trajectoy_bag.open(file_name, rosbag::bagmode::Read);
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
    return 1;
  }

  if (msgs_from_file.size() != 2)
  {
    ROS_ERROR_STREAM("Unable to import trajectory");
    return 1;
  }

  ram_msgs::AdditiveManufacturingTrajectory trajectory = msgs_from_file[0];
  if (trajectory.poses.size() == 0)
  {
    ROS_ERROR_STREAM("Trajectory is empty");
    return 1;
  }

  // Generate new UUIDs
  for (auto &p : trajectory.poses)
    p.unique_id = unique_id::toMsg(unique_id::fromRandom());

  // Save trajectory to bag file on disk
  std::string new_file_name(file_name);
  new_file_name.erase(new_file_name.end() - 4, new_file_name.end());
  new_file_name.append("_new.bag");

  // Save trajectory
  rosbag::Bag export_trajectoy_bag;
  try
  {
    export_trajectoy_bag.open(new_file_name, rosbag::bagmode::Write);
    export_trajectoy_bag.write("ram/trajectory", ros::Time::now(), trajectory);
    export_trajectoy_bag.write("ram/trajectory", ros::Time::now(), trajectory);
    export_trajectoy_bag.close();
  }
  catch (rosbag::BagException &e)
  {
    export_trajectoy_bag.close();
    ROS_ERROR_STREAM(e.what());
    return 1;
  }

  return 0;
}
