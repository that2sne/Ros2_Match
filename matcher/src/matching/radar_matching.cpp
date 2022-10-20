

#include "matcher/matching/radar_matching.hpp"

#include <cmath>

#include "matcher/fusion_algorithm/fusion_algorithm_factory.hpp"
#include "matcher/parameter/match_param_interface.hpp"
using namespace ebase::fusion::matcher;
RadarMatching::RadarMatching(RadarInfoBuffer & radar) : radar_buff_(radar) {}

RadarMatching::~RadarMatching() {}

void RadarMatching::AddData(MatchInfoPtrBuffer & match_info, int real_index)
{
  auto radar = radar_buff_.data[real_index];
  auto temporary_match_info = std::make_shared<MatchInfo>();
  temporary_match_info->fusion_id = fusion_id_->GetId();

  // radar data
  temporary_match_info->radar_id = radar.target_id;
  temporary_match_info->d_x = radar.longitudinal_x;
  temporary_match_info->d_y = radar.lateral_y;
  temporary_match_info->v_x = radar.v_x;
  temporary_match_info->v_y = radar.v_y;
  temporary_match_info->u = radar.u;
  temporary_match_info->v = radar.v;

  // For CarMaker
  temporary_match_info->d_z = radar.vertical_z;
  temporary_match_info->width = radar.width;
  temporary_match_info->a_x = radar.a_x;

  temporary_match_info->confidence = radar.confidence;
  temporary_match_info->motion_state = radar.motion_state;

  // OD data
  temporary_match_info->cam_id = -1;
  temporary_match_info->class_id = 0;
  temporary_match_info->bbox.x = 0;
  temporary_match_info->bbox.y = 0;
  temporary_match_info->bbox.width = 0;
  temporary_match_info->bbox.height = 0;

  // fusion Infomation
  temporary_match_info->sensor_type = SensorType::kRadarData;
  temporary_match_info->track_mode = TrackMode::kInit;
  temporary_match_info->match_flag = false;
  temporary_match_info->unmatch_time = 0;

  temporary_match_info->fusion =
    FusionAlgorithmFactory::CreateFusionAlgorithm(temporary_match_info);
  // temporary_match_info->fusion->Init();

  temporary_match_info->radar_longitudinal = radar.longitudinal_x;
  temporary_match_info->radar_lateral = radar.lateral_y;

  match_info.data.push_back(temporary_match_info);
}

void RadarMatching::InitializeData(MatchInfoPtrBuffer & match_info)
{
  for (int i = 0; i < radar_buff_.size(); i++) {
    AddData(match_info, i);
  }
}

bool RadarMatching::Update(MatchInfoPtrBuffer & match_buff_, int real_index, int match_index)
{
  RadarInfo & radar_info = radar_buff_.data[real_index];
  MatchInfo & match_info = *match_buff_.data[match_index];
  bool is_update = false;
  // Step 1: Checking case which estimation_id is same input_id
  if (match_info.radar_id == radar_info.target_id) {
    if (fabs(radar_info.longitudinal_x - match_info.d_x) <= MatchParamLongitGap()) {
      match_info.d_x = radar_info.longitudinal_x;
      match_info.d_y = radar_info.lateral_y;
      match_info.v_x = radar_info.v_x;
      match_info.v_y = radar_info.v_y;
      match_info.u = radar_info.u;
      match_info.v = radar_info.v;

      // For CarMaker
      match_info.d_z = radar_info.vertical_z;
      match_info.width = radar_info.width;
      match_info.a_x = radar_info.a_x;
      match_info.confidence = radar_info.confidence;
      match_info.motion_state = radar_info.motion_state;

      match_info.track_mode = TrackMode::kUpdate;
      match_info.match_flag = true;

      match_info.unmatch_time = 0;

      is_update = true;

      // @Debug
      match_info.radar_longitudinal = radar_info.longitudinal_x;
      match_info.radar_lateral = radar_info.lateral_y;
    }
  }
  return is_update;
}

bool RadarMatching::Match(
  MatchInfoPtrBuffer & match_buff_, int real_index, int match_index, float & min_distance)
{
  RadarInfo & radar_info = radar_buff_.data[real_index];
  MatchInfo & match_info = *match_buff_.data[match_index];
  bool is_match = false;
  // Step 1: Matching ROI 계산 - matching할 radar point를 탐색하는 영역 제한
  //         expendPixel * base_num^(-distance)
  int limit_region_x =
    (int)(match_info.bbox.width / (float)2 + 0.5) +
    (int)(MatchParamMaxSearchingRange() * pow(MatchParamBaseNum(), (-(radar_info.longitudinal_x / 100))));
  int limit_region_y =
    (int)(match_info.bbox.height / 2 + 0.5) +
    (int)(MatchParamMaxSearchingRange() * pow(MatchParamBaseNum(), (-(radar_info.longitudinal_x / 100))));

  // Step 2: Check radar point is located inside Matching ROI
  int od_center_x = (int)match_info.bbox.x + (int)(match_info.bbox.width / (float)2 + 0.5);
  int od_center_y = (int)match_info.bbox.y + (int)(match_info.bbox.height / (float)2 + 0.5);

  if (match_info.radar_id == radar_info.target_id) {
    min_distance = -1;

    match_info.radar_id = radar_info.target_id;
    match_info.d_x = radar_info.longitudinal_x;
    match_info.v_x = radar_info.v_x;
    match_info.v_y = radar_info.v_y;
    match_info.u = radar_info.u;
    match_info.v = radar_info.v;

    // For CarMaker
    match_info.d_z = radar_info.vertical_z;
    match_info.width = radar_info.width;
    match_info.a_x = radar_info.a_x;
    match_info.confidence = radar_info.confidence;
    match_info.motion_state = radar_info.motion_state;

    match_info.track_mode = TrackMode::kUpdate;
    match_info.match_flag = true;

    match_info.unmatch_time = 0;
    is_match = true;

    // @Debug
    match_info.radar_longitudinal = radar_info.longitudinal_x;
    match_info.radar_lateral = radar_info.lateral_y;
  } else if (
    min_distance != 0 && abs(od_center_x - radar_info.u) <= limit_region_x &&
    abs(od_center_y - radar_info.v) <= limit_region_y)
  {
    float distance = pow((od_center_x - radar_info.u), 2) + pow((od_center_y - radar_info.v), 2);
    distance = sqrt(distance);

    float threshold_longitudinal =
      radar_info.v_x == 0 ? 3.0 : radar_info.v_x * PeriodSensor::RADAR_PERIOD;
    threshold_longitudinal =
      match_info.sensor_type == SensorType::kFusionData ? threshold_longitudinal : 200;
    float diff_longitudinal = abs(match_info.d_x - radar_info.longitudinal_x);

    if (min_distance > distance && diff_longitudinal < threshold_longitudinal) {
      min_distance = distance;

      match_info.radar_id = radar_info.target_id;
      match_info.d_x = radar_info.longitudinal_x;
      match_info.v_x = radar_info.v_x;
      match_info.v_y = radar_info.v_y;
      match_info.u = radar_info.u;
      match_info.v = radar_info.v;

      // For CarMaker
      match_info.d_z = radar_info.vertical_z;
      match_info.width = radar_info.width;
      match_info.a_x = radar_info.a_x;
      match_info.confidence = radar_info.confidence;
      match_info.motion_state = radar_info.motion_state;

      match_info.track_mode = TrackMode::kUpdate;
      match_info.match_flag = true;

      match_info.unmatch_time = 0;
      is_match = true;

      // @Debug
      match_info.radar_longitudinal = radar_info.longitudinal_x;
      match_info.radar_lateral = radar_info.lateral_y;
    }
  }
  return is_match;
}
