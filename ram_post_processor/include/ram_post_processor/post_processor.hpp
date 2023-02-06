#ifndef RAM_POST_PROCESSOR_POST_PROCESSOR_HPP
#define RAM_POST_PROCESSOR_POST_PROCESSOR_HPP

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#include <eigen_conversions/eigen_msg.h>
#pragma GCC diagnostic pop

#include <fstream>
#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_post_processor/RobotProgram.h>
#include <ram_utils/GetStartPose.h>
#include <ram_utils/GetTool.h>
#include <ram_utils/GetTrajectoryInfos.h>
#include <ros/ros.h>
#include <sys/stat.h>

namespace ram_post_processor
{
  using NamedProgram = std::pair<std::string, std::string>;
  using NamedPrograms = std::vector<NamedProgram>;
  using ram_pose_const_iterator = ram_msgs::AdditiveManufacturingTrajectory::_poses_type::const_iterator;

  template<class ServiceReq, class ServiceRes>
  class PostProcessor
  {
  public:
    PostProcessor(const std::string name, const std::string description, const std::string service_name);
    virtual ~PostProcessor();
    virtual void addComment(const std::string &comment, ServiceRes &res);
    virtual void addPose(const bool,
                         const bool,
                         const bool,
                         const bool,
                         const bool,
                         const bool,
                         const ServiceReq &,
                         ServiceRes &);
    virtual void clearProgram(const ServiceReq &, ServiceRes &res);
    virtual void beforeGenerating(const ServiceReq &req, ServiceRes &res);
    virtual void afterGenerating(const ServiceReq &req, ServiceRes &res);
    // Polygons
    virtual void startPolygonBefore(const ServiceReq &, ServiceRes &res);
    virtual void startPolygonAfter(const ServiceReq &, ServiceRes &);
    virtual void finishPolygonBefore(const ServiceReq &, ServiceRes &);
    virtual void finishPolygonAfter(const ServiceReq &req, ServiceRes &res);
    // Layer change
    virtual void layerIndexChanged(const ServiceReq &req, ServiceRes &res);
    // Feed rate
    virtual void startFeedRateBefore(const ServiceReq &, ServiceRes &res);
    virtual void startFeedRateAfter(const ServiceReq &, ServiceRes &);
    virtual void changeFeedRateBefore(const ServiceReq &, ServiceRes &res);
    virtual void changeFeedRateAfter(const ServiceReq &, ServiceRes &);
    virtual void stopFeedRateBefore(const ServiceReq &, ServiceRes &res);
    virtual void stopFeedRateAfter(const ServiceReq &, ServiceRes &res);
    // Laser power
    virtual void startLaserPowerBefore(const ServiceReq &, ServiceRes &res);
    virtual void startLaserPowerAfter(const ServiceReq &, ServiceRes &);
    virtual void changeLaserPowerBefore(const ServiceReq &, ServiceRes &res);
    virtual void changeLaserPowerAfter(const ServiceReq &, ServiceRes &);
    virtual void stopLaserPowerBefore(const ServiceReq &, ServiceRes &res);
    virtual void stopLaserPowerAfter(const ServiceReq &, ServiceRes &);
    virtual void checkTrajectory(ServiceRes &res, ram_msgs::AdditiveManufacturingTrajectory &trajectory);
    virtual void checkPoses(ram_msgs::AdditiveManufacturingTrajectory &trajectory);
    virtual void transformAllPoses(ram_msgs::AdditiveManufacturingTrajectory &trajectory, Eigen::Isometry3d start_pose,
                                   Eigen::Isometry3d tool);
    virtual void startPoseService();
    virtual bool getTrajectoryInfo(double &seconds);
    virtual void callToolService();
    virtual void checkParameters(const ServiceReq &req, ServiceRes &res);
    virtual void addProgram(const ServiceReq &, ServiceRes &res, const std::string &name);
    virtual void addInformation(const ServiceReq &req, ServiceRes &res);
    virtual void processRequest(const ServiceReq &req, ServiceRes &res, ram_msgs::AdditiveManufacturingTrajectory &trajectory);
    virtual void saveToFiles(const std::string &directory, const std::string &file_extension, ServiceRes &res);

  public:
    const std::string name_;
    const std::string description_;
    const std::string service_name_;

    const ram_msgs::AdditiveManufacturingTrajectory &traj();
    const ram_pose_const_iterator &currentPose();

  protected:
    ros::NodeHandle nh_;
    Eigen::Isometry3d start_pose_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d tool_ = Eigen::Isometry3d::Identity();

    ram_msgs::AdditiveManufacturingTrajectory trajectory_;
    ram_pose_const_iterator current_pose_ = trajectory_.poses.begin();

    ros::ServiceClient get_start_pose_client_;
    ros::ServiceClient get_tool_client_;
    ros::ServiceClient get_trajectory_infos_client_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

}

#include <ram_post_processor/post_processor_impl.hpp>
#endif
