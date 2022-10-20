/**
 * @brief File for definition of class and member functions for match_process
 * @details In this file, Include files, variables,
 * structs which are in commonly used at the matching node are defined.
 * @author jimin cho
 * @date 2022.05.26
 * @version 0.1
 */

#ifndef FUSION_MATCH_PROCESS_HPP_
#define FUSION_MATCH_PROCESS_HPP_

#include <opencv2/opencv.hpp>

#include "common.hpp"
#include "enum_class.hpp"
#include "erae_fusion_msgs/msg/fusion_info_array.hpp"
#include "matcher/matching/matching.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace std;
using namespace cv;

namespace ebase
{
namespace fusion
{
namespace matcher
{

/**
 * @class MatchProcess class
 * @brief This class processes the matcing between radar and camera
 * @author jimin cho
 * @date 2022.05.29
 * @version 0.1
 */
class MatchProcess
{
public:
  /**
   * @fn MatchProcess()
   * @brief Constructor of class MatchProcess
   */
  MatchProcess(const rclcpp::Logger & logger);

public:
  /**
   * @fn int InitializeMatchBuff()
   * @brief When new scenario start, initialize and reset parameter, match buffer and msg
   * @return buffer size. when buffer size is not 0, it is error state
   */
  int InitializeMatchBuff();

  /**
   * @fn bool Execute(bool is_percept_empty, bool is_radar_empty)
   * @brief Matching main function, matching processor is started in this function
   * @param[in] is_percept_empty Variable indicating that the matching operation was operated
   * by the subscription of the perception node
   * @param[in] is_radar_empty Variable indicating that the matching operation was operated
   * by the subscription of the radar_sensing node
   * @return When the function operate the corrective action, return true. Otherwise, return false
   */
  bool Execute(std::unique_ptr<Matching> & exec);

  // private:
  /**
   * @fn int UpdateMatchBufferWithEstimate(const erae_fusion_msgs::msg::FusionInfoArray
   * estimation_buff)
   * @brief Update match_buffer as estimation data in buff
   * @param[in] estimation_buff_ estimated data from track_node
   * @return Match_buffer size
   */
  int UpdateMatchBufferWithEstimate(const erae_fusion_msgs::msg::FusionInfoArray & estimation_buff);

  void CleanUpUnnecessaryData();
  MatchInfoPtrBuffer & GetMatchBuf() { return match_buff_; }

private:
  /** @var MatchInfoBuffer match_buff_ buffer about matched data*/
  MatchInfoPtrBuffer match_buff_;

  FusionIdManager fusion_id_;

  void DebugLogging(shared_ptr<MatchInfo> debug_info);

  rclcpp::Logger logger_;

#ifdef _MATCH_EVALUATION
  string file_name;
  fstream log_file;
#endif
};

#define color_CAM Scalar(0, 255, 255)
#define color_RADAR Scalar(255, 0, 0)
#define color_ESTIMATION Scalar(0, 0, 255)
#define color_FUSION Scalar(185, 255, 183)

}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif
