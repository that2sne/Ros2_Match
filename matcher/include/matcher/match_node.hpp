/**
 * @file match_node.hpp
 * @author jimin cho
 * @brief File for definition of class and member functions for match_node
 * @version 0.1
 * @date 2022-05-20
 */

#ifndef FUSION_MATCH_NODE_HPP_
#define FUSION_MATCH_NODE_HPP_

#include <cv_bridge/cv_bridge.h>
#include <dirent.h>
#include <stdio.h>
#include <sys/stat.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// @Debug
//#include "erae_fusion_msgs/msg/match_debug_array.hpp"
//#include "erae_fusion_msgs/msg/match_in_out_debug.hpp"

#include "../../convert_coordinate/include/convert_coordinate/convert_process.hpp"
#include "enum_class.hpp"
#include "erae_diag_msgs/msg/module_sts.hpp"
#include "erae_vehicle_msgs/msg/vehicle_info.hpp"
#include "match_process.hpp"
#include "matcher/matching/matching.hpp"
#include "std_msgs/msg/string.hpp"
#include "ebase_node/ebase_node.hpp"
#include "erae_fusion_msgs/msg/draw_info.hpp"
#include "command.hpp"
//#include "erae_fusion_msgs/msg/fusion_info.hpp"

#define PI 3.141592

using namespace fusion::convert_process;
namespace ebase
{
namespace fusion
{
namespace matcher
{
/**
 * @class Matcher
 * @brief This class processes the following actions:
 * 1) Subscribe the real input data from radar sensor and perception node, and estimation data
 * from track node 2) Call the Matching algorithm class at regular intervals with real input data
 * and estimation data 3) Publish the match data to track node
 * @author jimin cho
 * @date 2022.05.20
 * @version 0.1
 */
class Matcher : public ebase::system::EBaseNode
{
public:
  /**
   * @fn void Matcher()
   * @details Constructor of class Matcher
   */
  Matcher();

private:
  /** @brief create timer callback */
  rclcpp::TimerBase::SharedPtr timer_;
  /// @brief Publisher declaration to publish to draw node. (Jimin)
  rclcpp::Publisher<erae_fusion_msgs::msg::DrawInfo>::SharedPtr publisher_draw_info;
  /** @brief Publish to track_node */
  rclcpp::Publisher<erae_fusion_msgs::msg::FusionInfoArray>::SharedPtr
    publisher_matchedInfo;  // to tracking node
  // @Debug
  // rclcpp::Publisher<erae_fusion_msgs::msg::MatchDebugArray>::SharedPtr
  //  publisher_debug;

  /** @brief Subscribe detected object messege from perception_node */
  rclcpp::Subscription<erae_perception_msgs::msg::FCW>::SharedPtr
    subscriber_bbox;  // from perception node
  /** @brief Subscribe detected target messege from radar_sensing_node */
  rclcpp::Subscription<erae_sensor_msgs::msg::MrrInfoArray>::SharedPtr
    subscriber_radar;  // from radar node
  /** @brief Subscribe estimated data messege from track_node */
  rclcpp::Subscription<erae_fusion_msgs::msg::FusionInfoArray>::SharedPtr
    subscriber_estiMatchedInfo;  // from tracking node
  /** @brief Subscriber declaration to subscribe CmInfo from CM_bridge. */
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_scenario_status;

  /**
   * @fn void SubscribeBboxCallback(const erae_perception_msgs::msg::FCW::SharedPtr msg)
   * @brief Callback function for receiving messege from perception node
   * @param[in] msg erar_perception_msgs/FCW messege consist of distance and bounding box of
   * detected object
   * @return void
   */
  void SubscribeBboxCallback(const erae_perception_msgs::msg::FCW::SharedPtr msg);
  /**
   * @fn void SubscribeRadarCallback(const erae_sensor_msgs::msg::MrrInfoArray::SharedPtr msg)
   * @brief Callback function for receiving messege from radar sensor node
   * @param[in] msg erae_sensor_msgs/MrrInfoArray messege consist of distance and velocity of
   * target
   * @return void
   */
  void SubscribeRadarCallback(const erae_sensor_msgs::msg::MrrInfoArray::SharedPtr msg);
  /**
   * @fn SubscribeEstimateCallback(const erae_fusion_msgs::msg::FusionInfoArray::SharedPtr msg)
   * @brief Callback function for receiving messege from track node
   * @param[in] msg erae_fusion_msgs/FusionInfoArray messege consist of estimation data from track
   * node
   * @return void
   */
  void SubscribeEstimateCallback(const erae_fusion_msgs::msg::FusionInfoArray::SharedPtr msg);

  bool SubscribeFromCmBridgeScenarioStatusCallback(const std_msgs::msg::String::SharedPtr msg);

  // For Demo: get vehicle information
  rclcpp::Subscription<erae_vehicle_msgs::msg::VehicleInfo>::SharedPtr subscriber_vehicle;
  void SubscribeVehicleInfo(const erae_vehicle_msgs::msg::VehicleInfo::SharedPtr msg);

public:
  // for marking recived msg from perception callback or radar callback
  ///  Bool type variable to indicate that perception data is received
  bool b_percep_callback;
  ///  Bool type variable to indicate that radar data is received
  bool b_radar_callback;

  /**
   * @fn int InitializeMsgData()
   * @brief When new scenario start, initialize and reset msg
   * @return buffer size. when buffer size is not 0, it is error state
   */
  // int InitializeMsgData();
  /**
   * @fn void MatchProcessor()
   * @brief Function that call the Matching algorithm process
   * @return void
   */
  void WorkerThread();
  /**
   * @fn bool CovertPerceptMsg2Struct(erae_perception_msgs::msg::FCW fcw_msg, PerceptInfoBuffer&
   * percept_buff_)
   * @brief Function that stores only the information required for matching among messages of
   * perception_node in perfection_buffer
   * @param[in] fcw_msg Messege which is received from perception_node
   * @param[out] percept_buff_ Type is struct, and this consists of bounding box of object,
   * distance
   * @return bool
   * When function worked well, function returns true. When there is an error in the function,
   * function returns false
   */
  bool CovertPerceptMsg2Struct(
    erae_perception_msgs::msg::FCW & fcw_msg, PerceptInfoBuffer & percept_buff_);

  /**
   * @fn bool ConvertRadarMsg2Struct(erae_sensor_msgs::msg::MrrInfoArray radar_msg,
   * RadarInfoBuffer& radar_buff_)
   * @brief Function that stores only the information required for matching among messages of
   * radar_node in radar_buffer
   * @param[in] radar_msg Messege which is received from radar_node
   * @param[out] radar_buff_ Type is struct, and this consists of distance
   * @return bool
   * When function worked well, function returns true. When there is an error in the function,
   * function returns false
   */
  bool ConvertRadarMsg2Struct(
    erae_sensor_msgs::msg::MrrInfoArray & radar_msg, RadarInfoBuffer & radar_buff_);

  /**
   * @fn bool ConvertFusionStruct2Msg(MatchInfoBuffer match_buff_,
   * erae_fusion_msgs::msg::FusionInfoArray& match_msg)
   * @brief FusionAlgorithm that copy the fusion_buffer to fusion_msg for publishing to track_node
   * @param[in] match_buff_ match_buff_ buffer is stored matched data
   * @param[out] match_msg publishing messege to track node
   * @return bool
   * When function worked well, function returns true. When there is an error in the function,
   * function returns false
   */
  bool ConvertFusionStruct2Msg(
    MatchInfoPtrBuffer & match_buff_, erae_fusion_msgs::msg::FusionInfoArray & match_msg);

  /**
   * @brief Function that stores perception data in DrawInfo message.
   * @param[in] fcw_msg Messege which is received from perception_node
   * @return void
   */
  void ConvertPerceptionMsg2DrawMsg(erae_perception_msgs::msg::FCW & fcw_msg);

  /**
   * @brief Function that stores radar data in DrawInfo message.
   * @param[in] radar_msg Messege which is received from radar_node
   * @return void
   */
  void ConvertRadarMsg2DrawMsg(erae_sensor_msgs::msg::MrrInfoArray & radar_msg);

  // control of exec_queue_
  void PushQueue(Command && exec);
  Command PopQueue();

private:
  // synchronization and control of thread
  std::thread thread_handle_;
  std::condition_variable cv_;
  std::mutex cv_m_;
  std::mutex exec_queue_m_;

  //
  std::list<Command> exec_queue_;

  // buffer about estimation_msg
  erae_fusion_msgs::msg::FusionInfoArray estimation_buff_;
  std::mutex estimation_buff_m_;

  // Data Input from each Sensor
  /** Messege which is received from track node */
  erae_fusion_msgs::msg::FusionInfoArray estimation_msg;

  erae_fusion_msgs::msg::DrawInfo draw_info_msg;

  // out msg to tracking
  /** Messege about matched data for publishing to track node */
  erae_fusion_msgs::msg::FusionInfoArray matched_msg;

  std_msgs::msg::String scenario_status_msg;

  // @Debug
  // erae_fusion_msgs::msg::MatchDebugArray debug_msg;

  /** Class for for performancing the matching function */
  std::unique_ptr<MatchProcess> matching_process_;
  /** Class for for performancing the conversion radar point to image plane */
  ConvertProcess c_convert_radar;
};

}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif
