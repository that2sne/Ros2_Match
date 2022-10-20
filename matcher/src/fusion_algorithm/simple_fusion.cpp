#include "matcher/fusion_algorithm/simple_fusion.hpp"

#include "matcher/common.hpp"
#include "matcher/enum_class.hpp"
#include "matcher/parameter/match_param_interface.hpp"

#include <algorithm>
using namespace ebase::fusion::matcher;
using namespace std;
using namespace chrono;
SimpleFusion::SimpleFusion(const std::weak_ptr<MatchInfo> & match_info)
: FusionAlgorithm(match_info),
  logger_(rclcpp::get_logger(string("Fusion ") + std::to_string(match_info_.lock()->fusion_id)))
{
  accumulation_time_ = 1000;
  max_cam_matching_ = accumulation_time_ / (PeriodSensor::CAMERA_PERIOD * 1000);
  max_radar_matching_ = accumulation_time_ / (PeriodSensor::RADAR_PERIOD * 1000);

  base_time_ = std::chrono::system_clock::now();
  accept_rate_ = 70.0f;
  delete_rate_ = 10.0f;
}

SimpleFusion::~SimpleFusion()
{
}

void SimpleFusion::TryFusion(SensorType && type)
{
  WriteMatchingInfo(type);
  if (IsAccumulationDone()) {
    UpdateStatus();
  }
}

void SimpleFusion::WriteMatchingInfo(SensorType & type)
{
  std::shared_ptr<MatchInfo> info = match_info_.lock();

#ifdef RAW_DATA
  debug_total_[static_cast<int>(type)]++;
  if (info->match_flag) debug_match_[static_cast<int>(type)]++;
#endif
  if (type == SensorType::kCameraData) {
    cam_.push_back(time_data{std::chrono::system_clock::now(), info->match_flag});
    EraseData(cam_, accumulation_time_ * 2);
  } else if (type == SensorType::kRadarData) {
    radar_.push_back(time_data{std::chrono::system_clock::now(), info->match_flag});
    EraseData(radar_, accumulation_time_ * 2);
  }
}

bool comp(const time_data & a, const system_clock::time_point & b)
{
  return a.first < b;
}

bool SimpleFusion::IsAccumulationDone()
{
  std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
  milliseconds duration = duration_cast<milliseconds>(current_time - base_time_);
  milliseconds accumulation(accumulation_time_);
  return duration >= accumulation;
}

void SimpleFusion::UpdateStatus()
{
  std::shared_ptr<MatchInfo> info = match_info_.lock();
  float radar_rate = GetMatchingRate(radar_, max_radar_matching_);
  float cam_rate = GetMatchingRate(cam_, max_cam_matching_);

#ifdef RAW_DATA
  if (radar_rate != 0 && cam_rate != 0) {
    RCLCPP_INFO(
      logger_, "DataInfo(matching/total) : Radar[%d/%d], Cam[%d/%d]", debug_match_[1],
      debug_total_[1], debug_match_[2], debug_total_[2]);
    RCLCPP_INFO(logger_, "Matching rate : Radar[%f], Cam[%f]", radar_rate, cam_rate);
  }
#endif
  RCLCPP_INFO(logger_, "Matching rate : Radar[%d], Cam[%d]", radar_.size(), cam_.size());
  if (
    radar_rate >= accept_rate_ && cam_rate >= accept_rate_ &&
    info->sensor_type != SensorType::kFusionData)
  {
    info->sensor_type = SensorType::kFusionData;
    base_time_ = std::chrono::system_clock::now() - chrono::milliseconds(500);
    RCLCPP_INFO(logger_, "Changed to fusion state.");
  } else if (
    radar_rate < accept_rate_ && cam_rate < accept_rate_ &&
    info->sensor_type == SensorType::kFusionData)
  {
    base_time_ = std::chrono::system_clock::now() - chrono::milliseconds(500);
    info->sensor_type = (radar_rate > cam_rate) ? SensorType::kRadarData : SensorType::kCameraData;
    RCLCPP_INFO(logger_, "The fusion state is broken.");
  }
  
  if (radar_rate < delete_rate_ && cam_rate < delete_rate_) {
    info->track_mode = TrackMode::kDelete;  // 삭제를 의미함.
    RCLCPP_INFO(logger_, "Will be deleted.");
  }
}

void SimpleFusion::EraseData(std::list<time_data> & data, const int keeping_time)
{
  auto effective_range = system_clock::now() - milliseconds(keeping_time);
  auto pos = std::lower_bound(data.begin(), data.end(), effective_range, comp);
  data.erase(data.begin(), pos);
}

float SimpleFusion::GetMatchingRate(const std::list<time_data> & data, int max_matching)
{
  // 유효 범위 산출
  auto effective_range = system_clock::now() - milliseconds(1000);
  auto pos = std::lower_bound(data.begin(), data.end(), effective_range, comp);

  // 매칭 횟수 계산
  int matching_count = 0;
  for (; pos != data.end(); pos++) {
    if (pos->second) matching_count++;
  }
  // 매칭 확률 계산
  float rate = (float)matching_count / max_matching * 100;
  return rate;
}
