#include "matcher/match_node.hpp"

#include "matcher/common.hpp"
#include "matcher/fusion_algorithm/fusion_algorithm_factory.hpp"

#include <functional>
#include <iostream>
#include <memory>
//#include "matcher/matching/percept_matching.hpp"
//#include "matcher/matching/radar_matching.hpp"
#include "matcher/parameter/match_parameter.hpp"
#include "matcher/percept_matching_command.hpp"
#include "matcher/radar_matching_command.hpp"
using namespace ebase::fusion::matcher;
using namespace std;
using std::placeholders::_1;
Matcher::Matcher() : EBaseNode("matcher", erae_diag_msgs::msg::ModuleSts::MODULE_FUSION_MATCH)
{
  // publish to rviz
  publisher_draw_info = this->create_publisher<erae_fusion_msgs::msg::DrawInfo>("match", 10);

  // from perception
  subscriber_bbox = this->create_subscription<erae_perception_msgs::msg::FCW>(
    "object_detection", 10, std::bind(&Matcher::SubscribeBboxCallback, this, _1));

  // from radar
  subscriber_radar = this->create_subscription<erae_sensor_msgs::msg::MrrInfoArray>(
    "mrr", 10, std::bind(&Matcher::SubscribeRadarCallback, this, _1));

  // from tracking node
  subscriber_estiMatchedInfo = this->create_subscription<erae_fusion_msgs::msg::FusionInfoArray>(
    "estimation", 10, std::bind(&Matcher::SubscribeEstimateCallback, this, _1));

  /// @brief CmInfo message is subscribed from CM_bridge.
  subscriber_scenario_status = this->create_subscription<std_msgs::msg::String>(
    "sim_status", 10, std::bind(&Matcher::SubscribeFromCmBridgeScenarioStatusCallback, this, _1));

  FusionAlgorithmFactory::getInstance()->SetAlgorithm(
    FusionAlgorithmFactory::FusionAlgorithmType::kSimpleFusion);

  // Thread initialization
  thread_handle_ = std::thread(&Matcher::WorkerThread, this);
}

void Matcher::SubscribeBboxCallback(const erae_perception_msgs::msg::FCW::SharedPtr msg)
{
  PushQueue(std::make_unique<PerceptMathcingCommand>(PerceptData(*msg)));
}

void Matcher::SubscribeRadarCallback(const erae_sensor_msgs::msg::MrrInfoArray::SharedPtr msg)
{
  PushQueue(std::make_unique<RadarMathcingCommand>(RadarData(*msg)));
}

void Matcher::SubscribeEstimateCallback(const erae_fusion_msgs::msg::FusionInfoArray::SharedPtr msg)
{
  estimation_buff_m_.lock();
  estimation_buff_ = *msg;
  estimation_buff_m_.unlock();
}

bool Matcher::SubscribeFromCmBridgeScenarioStatusCallback(
  const std_msgs::msg::String::SharedPtr msg)
{
  scenario_status_msg = *msg;

  if (scenario_status_msg.data == "start") {
    fusion_buf_.Clear();
  }

  return true;
}

void Matcher::PushQueue(std::unique_ptr<Command> && exec)
{
  std::lock(exec_queue_m_, cv_m_);
  std::lock_guard<std::mutex> lk1(exec_queue_m_, std::adopt_lock);
  std::lock_guard<std::mutex> lk2(cv_m_, std::adopt_lock);
  exec_queue_.push_back(std::move(exec));
  cv_.notify_all();
}

std::unique_ptr<Command> Matcher::PopQueue()
{
  std::lock_guard<std::mutex> lg(exec_queue_m_);
  if (exec_queue_.empty()) {
    return std::unique_ptr<Command>{};
  }
  std::unique_ptr<Command> cmd = std::move(exec_queue_.front());
  exec_queue_.pop_front();

  return cmd;
}

void Matcher::WorkerThread()
{
  while (1) {
    std::unique_ptr<Command> obj;
    {
      std::unique_lock<std::mutex> lk(cv_m_);

      obj = PopQueue();

      // std::this_thread::sleep_for(50ms);
      if (obj == nullptr) {
        cv_.wait(lk, [&] { return !exec_queue_.empty(); });
        continue;
      }
    }

    estimation_buff_m_.lock();
    fusion_buf_ += estimation_buff_;
    estimation_buff_m_.unlock();

    obj->exectue(fusion_buf_);

    erae_fusion_msgs::msg::DrawInfo draw_info_msg =
      static_cast<erae_fusion_msgs::msg::DrawInfo>(fusion_buf_);
    publisher_draw_info->publish(draw_info_msg);

    fusion_buf_.CleanUpUnnecessaryData();
  }
}