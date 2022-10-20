#include "matcher/fusion_algorithm/relative_speed_fusion.hpp"

#include "matcher/common.hpp"
#include "matcher/enum_class.hpp"
#include "matcher/parameter/match_param_interface.hpp"

#include <cmath>

using namespace ebase::fusion::matcher;
using namespace std;

RelativeSpeedFusion::RelativeSpeedFusion(const std::weak_ptr<MatchInfo> & match_info)
: FusionAlgorithm(match_info),
  radar_unmatching_(0.0),
  camera_unmatching_(0.0),
  current_status_(FusionMode::kReady),
  next_status_(FusionMode::kReady)
{
  std::shared_ptr<MatchInfo> info = match_info_.lock();
  if (info->sensor_type == SensorType::kRadarData) {
    radar_unmatching_ = MatchParamMaxMissingDistance();
  } else if (info->sensor_type == SensorType::kCameraData) {
    camera_unmatching_ = MatchParamMaxMissingDistance();
  }
}
RelativeSpeedFusion::~RelativeSpeedFusion()
{
}

void RelativeSpeedFusion::TryFusion(SensorType && type)
{
  std::shared_ptr<MatchInfo> info = match_info_.lock();
  if (info->match_flag == true && info->sensor_type == SensorType::kRadarData) {
    radar_unmatching_ = MatchParamMaxMissingDistance();
  } else if (info->match_flag == true && info->sensor_type == SensorType::kCameraData) {
    camera_unmatching_ = MatchParamMaxMissingDistance();
  }
  // When fusion mode is READY, this data can be change READY or FUSION or NONE(delete)
  if (current_status_ == FusionMode::kReady) {
    ChangeFromReadyState(*info, type);
  }
  // When fusion mode is FUSION, this data can be change FUSION or UNFUSION
  else if (current_status_ == FusionMode::kFusion)
  {
    ChangeFromFusionState(*info, type);
  }
  // When fusion mode is UNFUSION, this data can be change UNFUSION or FUSION or DELETE
  else if (current_status_ == FusionMode::kUnFusion)
  {
    ChangeFromUnFusionState(*info, type);
  }
}

FusionMode RelativeSpeedFusion::ChangeFromFusionState(
  MatchInfo & match_data, SensorType kind_of_input_data)
{
  FusionMode mode = FusionMode::kFusion;

  if (match_data.match_flag == true) {  // 현 callback에서 matching된 data
    mode = FusionMode::kFusion;
  } else {  // 현 callback에서 matching되지 않은 data
    float related_velocity = match_data.v_x;
    if (kind_of_input_data == SensorType::kCameraData) {
      float unmatching_distance = camera_unmatching_;
      if (unmatching_distance - abs(related_velocity * PeriodSensor::CAMERA_PERIOD) > 0) {
        mode = FusionMode::kFusion;
        camera_unmatching_ -= abs(related_velocity * PeriodSensor::CAMERA_PERIOD);
      } else {
        mode = FusionMode::kUnFusion;
        camera_unmatching_ = MatchParamMaxMissingDistance();
      }
    } else {
      float unmatching_distance = radar_unmatching_;
      if (unmatching_distance - abs(related_velocity * PeriodSensor::RADAR_PERIOD) > 0) {
        mode = FusionMode::kFusion;
        radar_unmatching_ -= abs(related_velocity * PeriodSensor::RADAR_PERIOD);
      } else {
        mode = FusionMode::kUnFusion;
        radar_unmatching_ = MatchParamMaxMissingDistance();
      }
    }
  }

  match_data.sensor_type = SensorType::kFusionData;
  return mode;
}

FusionMode RelativeSpeedFusion::ChangeFromUnFusionState(
  MatchInfo & match_data, SensorType kind_of_input_data)
{
  FusionMode mode = FusionMode::kUnFusion;

  float related_velocity = match_data.v_x;

  if (match_data.match_flag == true) {  // 현 callback에서 matching된 data
    // 우선은 상태 변이 time_gap 고려 X, 바로 fusion 상태로 변경
    mode = FusionMode::kFusion;
    match_data.sensor_type = SensorType::kFusionData;
  } else {  // 현 callback에서 matching되지 않은 data
    if (kind_of_input_data == SensorType::kCameraData) {
      float unmatching_distance = camera_unmatching_;
      if (unmatching_distance - abs(related_velocity * PeriodSensor::CAMERA_PERIOD) > 0) {
        mode = FusionMode::kUnFusion;
        camera_unmatching_ -= abs(related_velocity * PeriodSensor::CAMERA_PERIOD);
      } else {
        // mode = kReady;
        // match_data.sensor_type = kRadarData;
        mode = FusionMode::kNone;
        match_data.track_mode = TrackMode::kDelete;
        camera_unmatching_ = MatchParamMaxMissingDistance();
      }
    } else {
      float unmatching_distance = radar_unmatching_;
      if (unmatching_distance - abs(related_velocity * PeriodSensor::RADAR_PERIOD) > 0) {
        mode = FusionMode::kUnFusion;
        radar_unmatching_ -= abs(related_velocity * PeriodSensor::RADAR_PERIOD);
      } else {
        // mode = kReady;
        // match_data.sensor_type = kCameraData;
        mode = FusionMode::kNone;
        match_data.track_mode = TrackMode::kDelete;
        radar_unmatching_ = MatchParamMaxMissingDistance();
      }
    }
  }

  return mode;
}

FusionMode RelativeSpeedFusion::ChangeFromReadyState(
  MatchInfo & match_data, SensorType kind_of_input_data)
{
  FusionMode mode = FusionMode::kReady;

  // Camera only의 경우 상대 속도가 정보가 없음 How to???? --> Ego vehicle 속도 사용 변경해야함
  float related_velocity = match_data.v_x != 0 ? match_data.v_x : 10.0;

  if (match_data.match_flag == true) {  // 현 callback에서 matching된 data
    // 우선은 상태 변이 time_gap 고려 X, 바로 fusion 상태로 변경
    mode = FusionMode::kFusion;
    match_data.sensor_type = SensorType::kFusionData;
  } else {  // 현 callback에서 matching되지 않은 data
    if (kind_of_input_data == SensorType::kCameraData) {
      float unmatching_distance = camera_unmatching_;
      // 상태 변이 유지 거리: 5m -> 5m간 매칭이 일어나지 않으면 삭제
      // 매칭이 안되면 유지거리에서 상대 속도로 계산된 거리 차감
      if (unmatching_distance - abs(related_velocity * PeriodSensor::CAMERA_PERIOD) > 0) {
        mode = FusionMode::kReady;
        match_data.sensor_type = SensorType::kCameraData;
        camera_unmatching_ -= abs(related_velocity * PeriodSensor::CAMERA_PERIOD);
      } else {
        mode = FusionMode::kNone;
        match_data.track_mode = TrackMode::kDelete;
        camera_unmatching_ = MatchParamMaxMissingDistance();
      }
    } else {
      float unmatching_distance = radar_unmatching_;
      if (unmatching_distance - abs(related_velocity * PeriodSensor::RADAR_PERIOD) > 0) {
        mode = FusionMode::kReady;
        match_data.sensor_type = SensorType::kRadarData;
        radar_unmatching_ -= abs(related_velocity * PeriodSensor::RADAR_PERIOD);
      } else {
        mode = FusionMode::kNone;
        match_data.track_mode = TrackMode::kDelete;
        radar_unmatching_ = MatchParamMaxMissingDistance();
      }
    }
  }

  return mode;
}
