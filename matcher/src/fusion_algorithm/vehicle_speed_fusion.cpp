#include "matcher/fusion_algorithm/vehicle_speed_fusion.hpp"

#include "matcher/common.hpp"
#include "matcher/enum_class.hpp"
#include "matcher/parameter/match_param_interface.hpp"

#include <chrono>
#include <cmath>

using namespace ebase::fusion::matcher;
using namespace std;
using namespace chrono;

// @jm: 스레드 구현 후, timeout_cnt 제거(for 횟수 기반)
VehicleSpeedFusion::VehicleSpeedFusion(const std::weak_ptr<MatchInfo> & match_info)
: /*holding_time_(0), timeout_cnt(0) */
  FusionAlgorithm(match_info),
  cam_holding_time_(0),
  radar_holding_time_(0),
  cam_count_(0),
  radar_count_(0),
  cam_total_count_(0),
  radar_total_count_(0),
  next_type_(SensorType::kError),
  current_status_(FusionMode::kReady),
  next_status_(FusionMode::kReady)
{
  CalculateHoldingTime();
}
VehicleSpeedFusion::~VehicleSpeedFusion()
{
}

FusionMode VehicleSpeedFusion::GetStatus()
{
  return current_status_;
}
FusionMode VehicleSpeedFusion::GetNextStatus()
{
  return next_status_;
}

void VehicleSpeedFusion::TryFusion(SensorType && type)
{
  if (IsChanging()) {
    CheckProgress(type);
  } else {
    TryChange(type);
  }
}
// 현재 fusion 상태 변화 중인지 아닌지 확인하는 함수
bool VehicleSpeedFusion::IsChanging()
{
  // fusion 상태를 유지해야하는 상황이면 true, 상태가 변하는 상황이면 false
  bool result = false;

  if (next_status_ == current_status_)
    result = false;
  else
    result = true;

  return result;
  // return next_status_ != current_status_;
}

bool VehicleSpeedFusion::IsTimeout(SensorType & type)
{
  // @jm: 우선은 unmatching 횟수로 알고리즘 구현, 스레드 구현 후 시간 조건 넣어야 함
  /*milliseconds duration = duration_cast<milliseconds>(end - start_);
  milliseconds holding = duration_cast<milliseconds>(holding_time_);*/

  int duration = (type == SensorType::kRadarData) ? radar_total_count_ : cam_total_count_;
  int holding = (type == SensorType::kRadarData) ? radar_holding_time_ : cam_holding_time_;

  if (next_status_ == FusionMode::kNone)
    holding *= 0.7;
  else if (next_status_ == FusionMode::kFusion)
    holding *= 0.5;

  return duration >= holding;
}

// 상태를 바꿀지 말지 결정하는 함수
void VehicleSpeedFusion::CheckProgress(SensorType & type)
{
  std::shared_ptr<MatchInfo> info = match_info_.lock();
  // @jm: 검토 후, 매칭률 임계값 파라미터로 변경
  float accept_rate =
    (next_status_ == FusionMode::kFusion) ? MatchParamMatchingRate() : MatchParamUnMatchingRate();

  // 1. 매칭률 연산
  float match_rate = 0.0;
  if (type == SensorType::kRadarData) {
    match_rate = ((float)radar_count_ / (float)radar_total_count_) * 100;
  } else {
    match_rate = ((float)cam_count_ / (float)cam_total_count_) * 100;
  }

  // 2. 체크 시간 확인
  if (IsTimeout(type)) {
    if (match_rate >= accept_rate) {
      // end time check
      // auto end = std::chrono::system_clock::now();
      current_status_ = next_status_;

      if (current_status_ == FusionMode::kReady) {
        if (type == SensorType::kRadarData) {
          info->radar_id = -1;
        } else {
          info->cam_id = -1;
        }
      } else if (current_status_ == FusionMode::kUnFusion) {
        if (type == SensorType::kRadarData) {
          next_type_ = SensorType::kCameraData;
        } else {
          next_type_ = SensorType::kRadarData;
        }
      }
    } else {
      next_status_ = current_status_;
    }

    // Change Sensor_Type
    info->sensor_type = ChangeSensorType(*info);

    radar_count_ = radar_total_count_ = 1;
    cam_count_ = cam_total_count_ = 1;
  } else {
    if (type == SensorType::kRadarData) {
      if (
        (info->match_flag == true && next_status_ == FusionMode::kFusion) ||
        (info->match_flag == false && next_status_ != FusionMode::kFusion))
      {
        radar_count_++;
      }
      radar_total_count_++;
    } else {
      if (
        (info->match_flag == true && next_status_ == FusionMode::kFusion) ||
        (info->match_flag == false && next_status_ != FusionMode::kFusion))
      {
        cam_count_++;
      }
      cam_total_count_++;
    }
  }
}

SensorType VehicleSpeedFusion::ChangeSensorType(MatchInfo & info)
{
  SensorType type = SensorType::kError;
  if (
    current_status_ == FusionMode::kFusion ||
    current_status_ == FusionMode::kUnFusion)
  {
    type = SensorType::kFusionData;
  } else if (current_status_ == FusionMode::kReady) {
    if (next_type_ == SensorType::kRadarData) {
      type = SensorType::kRadarData;
    } else {
      type = SensorType::kCameraData;
    }
  } else if (current_status_ == FusionMode::kNone) {
    info.track_mode = TrackMode::kDelete;
    // type = SensorType::kError;
  }

  return type;
}

// 다음 fusion 상태 및 holding_time 결정을 수행하는 함수
void VehicleSpeedFusion::TryChange(SensorType & type)
{
  std::shared_ptr<MatchInfo> info = match_info_.lock();
  radar_count_ = radar_total_count_ = 1;
  cam_count_ = cam_total_count_ = 1;
  // 다음 상태와 확인할 시간(ms)를 리턴해야함. (최초 상태 변화 발생 시, 계산)
  // (22.09.08) holding_time를 자차 속도에 따른 시간이 아닌 고정된 time sequence(횟수)로 변경
  // holding_time = CalculateHoldingTime(ego_speed);

  // FusionMode next_status;
  next_status_ = AssignNextStatus(*info, type);

  // fusion_mode가 변경된 시간
  start_ = std::chrono::system_clock::now();
}

/*int VehicleSpeedFusion::CalculateHoldingTime(double ego_speed)
{
  //ego_speed 기반 holding_time 계산
  int holding_time = (int)((MatchParamMaxMissingDistance() / ego_speed) * 1000);

  return holding_time;
}*/

int VehicleSpeedFusion::CalculateHoldingTime()
{
  // @jm: int duration를 parameter로 변경
  float duration = MatchParamDuration();
  cam_holding_time_ = (int)(duration / (float)PeriodSensor::CAMERA_PERIOD);
  radar_holding_time_ = (int)(duration / (float)PeriodSensor::RADAR_PERIOD);
  return 0;
}

FusionMode VehicleSpeedFusion::AssignNextStatus(MatchInfo & info, SensorType & type)
{
  FusionMode mode = FusionMode::kNone;

  if (current_status_ == FusionMode::kReady) {
    mode = ChangeFromReadyState(info, type);
  }
  // When fusion mode is FUSION, this data can be change FUSION or UNFUSION
  else if (current_status_ == FusionMode::kFusion)
  {
    mode = ChangeFromFusionState(info, type);
  }
  // When fusion mode is UNFUSION, this data can be change UNFUSION or FUSION or DELETE
  else if (current_status_ == FusionMode::kUnFusion)
  {
    mode = ChangeFromUnFusionState(info, type);
  }

  return mode;
}

FusionMode VehicleSpeedFusion::ChangeFromReadyState(MatchInfo & info, SensorType type)
{
  FusionMode mode = FusionMode::kReady;

  if (info.match_flag == true) {  // 현 callback에서 matching 혹은 update된 data
    if (info.radar_id != -1 && info.cam_id != -1) {  // 두 센서 데이터가 matching된 데이터
      mode = FusionMode::kFusion;
    } else {  // 카메라/레이터 only 데이터가 업데이트 된 경우
      mode = FusionMode::kReady;
    }
  } else {
    // Can be change delete
    mode = FusionMode::kNone;
  }

  return mode;
}

FusionMode VehicleSpeedFusion::ChangeFromFusionState(MatchInfo & info, SensorType type)
{
  FusionMode mode = FusionMode::kFusion;

  if (info.match_flag) {
    // Can be change kFusion
    mode = FusionMode::kFusion;
  } else {
    // Can be change kUnFusion
    mode = FusionMode::kUnFusion;
    // mode = FusionMode::kReady;
  }

  return mode;
}

FusionMode VehicleSpeedFusion::ChangeFromUnFusionState(MatchInfo & info, SensorType type)
{
  FusionMode mode = FusionMode::kUnFusion;

  // @jm: UnFusion state 우선 보류
  if (info.match_flag) {
    // Can be change kUnFusion or kFusion
    if (next_type_ != type) {  // 잔류한 data와 matching된 input data type이 다른 경우
      mode = FusionMode::kFusion;
    } else {  // 잔류한 data와 동일한 센서의 input data가 업데이트 된 경우
      mode = FusionMode::kUnFusion;
    }
  } else {
    // Can be change kReady
    mode = FusionMode::kReady;
  }
  return mode;
}
