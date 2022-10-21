/**
 * @file match_node.hpp
 * @author jimin cho
 * @brief File for definition of class and member functions for match_node
 * @version 0.1
 * @date 2022-05-20
 */

#ifndef FUSION_MATCH_NODE_HPP_
#define FUSION_MATCH_NODE_HPP_

#include <algorithm>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "command.hpp"
#include "erae_diag_msgs/msg/module_sts.hpp"
#include "ebase_node/ebase_node.hpp"
#include "enum_class.hpp"

#include "erae_fusion_msgs/msg/draw_info.hpp"
#include "erae_fusion_msgs/msg/fusion_info_array.hpp"
#include "erae_perception_msgs/msg/fcw.hpp"
#include "erae_sensor_msgs/msg/mrr_info_array.hpp"
//#include "erae_vehicle_msgs/msg/vehicle_info.hpp"
#include "matcher/fusion_data.hpp"
#include "std_msgs/msg/string.hpp"

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
  /// @brief Publisher declaration to publish to draw node. (Jimin)
  rclcpp::Publisher<erae_fusion_msgs::msg::DrawInfo>::SharedPtr publisher_draw_info;

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

public:
  void WorkerThread();
  // control of exec_queue_
  void PushQueue(std::unique_ptr<Command> && exec);
  std::unique_ptr<Command> PopQueue();

private:
  // synchronization and control of thread
  std::thread thread_handle_;
  std::condition_variable cv_;
  std::mutex cv_m_;
  std::mutex exec_queue_m_;

  //
  std::list<std::unique_ptr<Command>> exec_queue_;

  // buffer about estimation_msg
  erae_fusion_msgs::msg::FusionInfoArray estimation_buff_;
  std::mutex estimation_buff_m_;

  std_msgs::msg::String scenario_status_msg;

  // New
  FusionData fusion_buf_;
};

}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif
