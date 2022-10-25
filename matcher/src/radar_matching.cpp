#include "matcher/matching.hpp"
#include "matcher/parameter/match_param_interface.hpp"

using ebase::fusion::matcher::IMatch;
using ebase::fusion::matcher::Matching;
using ebase::fusion::matcher::RadarData;
using ebase::fusion::matcher::RadarInfo;
using ebase::fusion::matcher::RadarMatching;

RadarMatching::RadarMatching(const rclcpp::Clock::SharedPtr & clock, const rclcpp::Logger & logger)
: clock_(clock), logger_(logger)
{
  match_funs_.Update = std::bind(
    &RadarMatching::Update, std::ref(*this), std::placeholders::_1, std::placeholders::_2);
  match_funs_.Match = std::bind(
    &RadarMatching::Match, std::ref(*this), std::placeholders::_1, std::placeholders::_2,
    std::placeholders::_3);
}

void RadarMatching::Process(FusionData & fd, const RadarData & input_data)
{
  Matching::Process(fd, input_data, match_funs_);
}

bool RadarMatching::Match(MatchInfo & mi, const RadarInfo & ri, float & min_distance)
{
  bool is_match = false;
  // Step 1: Matching ROI 계산 - matching할 radar point를 탐색하는 영역 제한
  //         expendPixel * base_num^(-distance)
  int limit_region_x =
    (int)(mi.bbox.width / (float)2 + 0.5) +
    (int)(MatchParamMaxSearchingRange() * pow(MatchParamBaseNum(), (-(ri.longitudinal_x / 100))));
  int limit_region_y =
    (int)(mi.bbox.height / 2 + 0.5) +
    (int)(MatchParamMaxSearchingRange() * pow(MatchParamBaseNum(), (-(ri.longitudinal_x / 100))));

  // Step 2: Check radar point is located inside Matching ROI
  int od_center_x = (int)mi.bbox.x + (int)(mi.bbox.width / (float)2 + 0.5);
  int od_center_y = (int)mi.bbox.y + (int)(mi.bbox.height / (float)2 + 0.5);

  if (mi.radar_id == ri.target_id) {
    // if (mi.cam_id == ri.target_id) {
    min_distance = -1;

    mi.radar_id = ri.target_id;
    mi.d_x = ri.longitudinal_x;
    mi.v_x = ri.v_x;
    mi.v_y = ri.v_y;
    mi.u = ri.u;
    mi.v = ri.v;

    // For CarMaker
    mi.d_z = ri.vertical_z;
    mi.width = ri.width;
    mi.a_x = ri.a_x;
    mi.confidence = ri.confidence;
    mi.motion_state = ri.motion_state;

    mi.track_mode = TrackMode::kUpdate;
    mi.match_flag = true;

    mi.unmatch_time = 0;
    is_match = true;

    // @Debug
    mi.radar_longitudinal = ri.longitudinal_x;
    mi.radar_lateral = ri.lateral_y;
  } else if (
    min_distance != 0 && abs(od_center_x - ri.u) <= limit_region_x &&
    abs(od_center_y - ri.v) <= limit_region_y)
  {
    float distance = pow((od_center_x - ri.u), 2) + pow((od_center_y - ri.v), 2);
    distance = sqrt(distance);

    float threshold_longitudinal = ri.v_x == 0 ? 3.0 : ri.v_x * PeriodSensor::RADAR_PERIOD;
    threshold_longitudinal =
      mi.sensor_type == SensorType::kFusionData ? threshold_longitudinal : 200;
    float diff_longitudinal = abs(mi.d_x - ri.longitudinal_x);

    if (min_distance > distance && diff_longitudinal < threshold_longitudinal) {
      min_distance = distance;

      mi.radar_id = ri.target_id;
      mi.d_x = ri.longitudinal_x;
      mi.v_x = ri.v_x;
      mi.v_y = ri.v_y;
      mi.u = ri.u;
      mi.v = ri.v;

      // For CarMaker
      mi.d_z = ri.vertical_z;
      mi.width = ri.width;
      mi.a_x = ri.a_x;
      mi.confidence = ri.confidence;
      mi.motion_state = ri.motion_state;

      mi.track_mode = TrackMode::kUpdate;
      mi.match_flag = true;

      mi.unmatch_time = 0;
      is_match = true;

      // @Debug
      mi.radar_longitudinal = ri.longitudinal_x;
      mi.radar_lateral = ri.lateral_y;
      printf(
        "[Fusion %d]New matching has occurred. R:%d C:%d\n", mi.fusion_id, ri.target_id, mi.cam_id);
      fflush(stdout);
    }
  }
  return is_match;
}

// 단일 개체에 대한 업데이트
bool RadarMatching::Update(MatchInfo & mi, const RadarInfo & ri)
{
  bool is_update = false;
  // Step 1: Checking case which estimation_id is same input_id
  if (mi.radar_id == ri.target_id) {
    if (fabs(ri.longitudinal_x - mi.d_x) <= MatchParamLongitGap()) {
      mi.d_x = ri.longitudinal_x;
      mi.d_y = ri.lateral_y;
      mi.v_x = ri.v_x;
      mi.v_y = ri.v_y;
      mi.u = ri.u;
      mi.v = ri.v;

      // For CarMaker
      mi.d_z = ri.vertical_z;
      mi.width = ri.width;
      mi.a_x = ri.a_x;
      mi.confidence = ri.confidence;
      mi.motion_state = ri.motion_state;

      mi.track_mode = TrackMode::kUpdate;
      mi.match_flag = true;

      mi.unmatch_time = 0;

      is_update = true;

      // @Debug
      mi.radar_longitudinal = ri.longitudinal_x;
      mi.radar_lateral = ri.lateral_y;
    }
  }
  return is_update;
}
