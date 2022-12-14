#include "matcher/fusion_algorithm/simple_fusion.hpp"

#include "matcher/common.hpp"
#include "matcher/enum_class.hpp"
#include "matcher/parameter/match_param_interface.hpp"

#include <algorithm>
using namespace ebase::fusion::matcher;
using namespace std;
using namespace chrono;
SimpleFusion::SimpleFusion(const rclcpp::Clock::SharedPtr & clock, const rclcpp::Logger & logger)
: clock_(clock), logger_(logger)
{
  accumulation_time_ = 1000;
  max_cam_matching_ = accumulation_time_ / (PeriodSensor::CAMERA_PERIOD * 1000);
  max_radar_matching_ = accumulation_time_ / (PeriodSensor::RADAR_PERIOD * 1000);

  // base_time_ = std::chrono::system_clock::now();
  accept_rate_ = 70.0f;
  delete_rate_ = 10.0f;
}

SimpleFusion::~SimpleFusion()
{
}

void SimpleFusion::TryFusion(FusionData & fd, SensorType && type)
{
  for (auto & mi : fd) {
    // data_[i.fusion_id] = empty 면 초기화
    if (data_.find(mi.fusion_id) == data_.end()) {
      data_[mi.fusion_id].base_time_ = std::chrono::system_clock::now();
      RCLCPP_INFO(logger_, "[%d] added R[%d] : C [%d]", mi.fusion_id, mi.radar_id, mi.cam_id);
    }
    WriteMatchingInfo(data_[mi.fusion_id], mi, type);
    if (IsAccumulationDone(data_[mi.fusion_id])) {
      UpdateStatus(data_[mi.fusion_id], mi);
    }
  }
}
void SimpleFusion::Delete(int fusion_id)
{
  auto it = data_.find(fusion_id);
  if (it != data_.end()) {
    data_.erase(it);
    RCLCPP_INFO(logger_, "[%d] deleted.", fusion_id);
  }
}
//
void SimpleFusion::WriteMatchingInfo(
  Accumulation_data & dt, const MatchInfo & mi, const SensorType & type)
{
#ifdef RAW_DATA
  debug_total_[static_cast<int>(type)]++;
  if (info->match_flag) debug_match_[static_cast<int>(type)]++;
#endif
  if (type == SensorType::kCameraData) {
    dt.cam_.push_back(time_data{std::chrono::system_clock::now(), mi.match_flag});
    EraseData(dt.cam_, accumulation_time_ * 2);
  } else if (type == SensorType::kRadarData) {
    dt.radar_.push_back(time_data{std::chrono::system_clock::now(), mi.match_flag});
    EraseData(dt.radar_, accumulation_time_ * 2);
  }
}

bool comp(const time_data & a, const system_clock::time_point & b)
{
  return a.first < b;
}

bool SimpleFusion::IsAccumulationDone(Accumulation_data & dt)
{
  std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
  milliseconds duration = duration_cast<milliseconds>(current_time - dt.base_time_);
  milliseconds accumulation(accumulation_time_);
  return duration >= accumulation;
}

void SimpleFusion::UpdateStatus(Accumulation_data & dt, MatchInfo & mi)
{
  float radar_rate = GetMatchingRate(dt.radar_, max_radar_matching_);
  float cam_rate = GetMatchingRate(dt.cam_, max_cam_matching_);

#ifdef RAW_DATA
  if (radar_rate != 0 && cam_rate != 0) {
    RCLCPP_INFO(
      logger_, "DataInfo(matching/total) : Radar[%d/%d], Cam[%d/%d]", debug_match_[1],
      debug_total_[1], debug_match_[2], debug_total_[2]);
    RCLCPP_INFO(logger_, "Matching rate : Radar[%f], Cam[%f]", radar_rate, cam_rate);
  }
#endif
  // RCLCPP_INFO(logger_, "Matching rate : Radar[%f], Cam[%f]", radar_rate, cam_rate);
  //  RCLCPP_INFO(logger_, "Matching rate : Radar[%d], Cam[%d]", dt.radar_.size(), dt.cam_.size());
  if (
    radar_rate >= accept_rate_ && cam_rate >= accept_rate_ &&
    mi.sensor_type != SensorType::kFusionData)
  {
    mi.sensor_type = SensorType::kFusionData;
    dt.base_time_ = std::chrono::system_clock::now() - chrono::milliseconds(500);
    RCLCPP_INFO(logger_, "[%d] Changed to fusion state.", mi.fusion_id);
  } else if (
    radar_rate < accept_rate_ && cam_rate < accept_rate_ &&
    mi.sensor_type == SensorType::kFusionData)
  {
    dt.base_time_ = std::chrono::system_clock::now() - chrono::milliseconds(500);
    mi.sensor_type = (radar_rate > cam_rate) ? SensorType::kRadarData : SensorType::kCameraData;
    RCLCPP_INFO(logger_, "[%d] The fusion state is broken.", mi.fusion_id);
  }

  if (radar_rate < delete_rate_ && cam_rate < delete_rate_) {
    mi.track_mode = TrackMode::kDelete;  // 삭제를 의미함.
    RCLCPP_INFO(logger_, "[%d] Will be deleted.", mi.fusion_id);
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
