#ifndef RAM_POST_PROCESSOR_CSV_POST_PROCESSOR_HPP
#define RAM_POST_PROCESSOR_CSV_POST_PROCESSOR_HPP

#include <ram_post_processor/post_processor.hpp>
#include <ram_post_processor/GenerateCSV.h>
#include <industrial_robot_angle_conversions/poses.hpp>

namespace ram_post_processor
{
class CSVPostProcessor : public ram_post_processor::PostProcessor
    <ram_post_processor::GenerateCSVRequest, ram_post_processor::GenerateCSVResponse>
{
public:
  CSVPostProcessor();

  virtual ~CSVPostProcessor()
  {
  }

  void addPose(const bool,
               const bool,
               const bool,
               const bool,
               const bool,
               const bool,
               const ram_post_processor::GenerateCSVRequest &req,
               ram_post_processor::GenerateCSVResponse &res);

  void addInformation(const ram_post_processor::GenerateCSVRequest &req,
                      ram_post_processor::GenerateCSVResponse &res);

  void afterGenerating(const ram_post_processor::GenerateCSVRequest &req,
                       ram_post_processor::GenerateCSVResponse &res);

  void layerIndexChanged(const ram_post_processor::GenerateCSVRequest &req,
                         ram_post_processor::GenerateCSVResponse &res);

  void startFeedRateBefore(const ram_post_processor::GenerateCSVRequest &,
                           ram_post_processor::GenerateCSVResponse &res);

  void startPolygonBefore(const ram_post_processor::GenerateCSVRequest &,
                          ram_post_processor::GenerateCSVResponse &res);

  void finishPolygonAfter(const ram_post_processor::GenerateCSVRequest &req,
                          ram_post_processor::GenerateCSVResponse &res);

  void changeFeedRateBefore(const ram_post_processor::GenerateCSVRequest &,
                            ram_post_processor::GenerateCSVResponse &res);

  void stopFeedRateBefore(const ram_post_processor::GenerateCSVRequest &,
                          ram_post_processor::GenerateCSVResponse &res);

  void stopFeedRateAfter(const ram_post_processor::GenerateCSVRequest &,
                         ram_post_processor::GenerateCSVResponse &res);

  void startLaserPowerBefore(const ram_post_processor::GenerateCSVRequest &,
                             ram_post_processor::GenerateCSVResponse &res);

  void changeLaserPowerBefore(const ram_post_processor::GenerateCSVRequest &,
                              ram_post_processor::GenerateCSVResponse &res);

  void stopLaserPowerBefore(const ram_post_processor::GenerateCSVRequest &,
                            ram_post_processor::GenerateCSVResponse &res);

  void beforeGenerating(const ram_post_processor::GenerateCSVRequest &req,
                        ram_post_processor::GenerateCSVResponse &res);

private:
  int64_t pose_number_ = 1;
};

}
#endif
