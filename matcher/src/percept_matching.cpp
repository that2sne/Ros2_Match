#include "matcher/matching.hpp"
#include "matcher/fusion_data.hpp"
#include "matcher/parameter/match_param_interface.hpp"
using ebase::fusion::matcher::IMatch;
using ebase::fusion::matcher::Matching;
using ebase::fusion::matcher::PerceptData;
using ebase::fusion::matcher::PerceptInfo;
using ebase::fusion::matcher::PerceptMatching;

PerceptMatching::PerceptMatching(
  const rclcpp::Clock::SharedPtr & clock, const rclcpp::Logger & logger)
: clock_(clock), logger_(logger)
{
  match_funs_.Update = std::bind(
    &PerceptMatching::Update, std::ref(*this), std::placeholders::_1, std::placeholders::_2);
  match_funs_.Match = std::bind(
    &PerceptMatching::Match, std::ref(*this), std::placeholders::_1, std::placeholders::_2,
    std::placeholders::_3);
}

void PerceptMatching::Process(FusionData & fd, const PerceptData & input_data)
{
  Matching::Process(fd, input_data, match_funs_);
}

bool PerceptMatching::Match(MatchInfo & mi, const PerceptInfo & pi, float & min_distance)
{
  bool is_match = false;
  // Step 1: Matching ROI 계산 - matching할 radar point를 탐색하는 영역 제한
  //          * base_num^(-distance)
  int limit_region_x =
    (int)(pi.width / (float)2 + 0.5) +
    (int)(MatchParamMaxSearchingRange() * pow(MatchParamBaseNum(), (-(mi.d_x / 100))));
  int limit_region_y =
    (int)(pi.height / (float)2 + 0.5) +
    (int)(MatchParamMaxSearchingRange() * pow(MatchParamBaseNum(), (-(mi.d_x / 100))));

  // Step 2: Check radar point is located inside Matching ROI
  int od_center_x = (int)pi.x + (int)(pi.width / (float)2 + 0.5);
  int od_center_y = (int)pi.y + (int)(pi.height / (float)2 + 0.5);

  if (mi.cam_id == pi.obj_id) {
    // if (mi.radar_id == pi.obj_id) {
    min_distance = -1;

    // For CarMaker
    mi.class_id = pi.class_id;

    mi.cam_id = pi.obj_id;
    mi.bbox.x = pi.x;
    mi.bbox.y = pi.y;
    mi.bbox.height = pi.height;
    mi.bbox.width = pi.width;

    mi.u = mi.u;
    mi.v = mi.v;

    // For CarMaker
    mi.d_y = pi.lateral;
    mi.d_z = pi.vertical;
    mi.width = pi.width;

    mi.track_mode = TrackMode::kUpdate;
    mi.match_flag = true;

    mi.unmatch_time = 0;

    is_match = true;

    // @Debug
    mi.radar_longitudinal = pi.longitudinal;
    mi.radar_lateral = pi.lateral;
  } else if (abs(od_center_x - mi.u) <= limit_region_x && abs(od_center_y - mi.v) <= limit_region_y)
  {
    float distance = pow((od_center_x - mi.u), 2) + pow((od_center_y - mi.v), 2);
    distance = sqrt(distance);

    float threshold_longitudinal = mi.v_x == 0 ? 3.0 : mi.v_x * PeriodSensor::CAMERA_PERIOD;
    threshold_longitudinal =
      mi.sensor_type == SensorType::kFusionData ? threshold_longitudinal : 200;
    float diff_longitudinal = abs(mi.d_x - pi.longitudinal);

    // Step 3. Find the closest radar point to Bbox center
    if (distance < min_distance && diff_longitudinal < threshold_longitudinal) {
      min_distance = distance;
      // For CarMaker
      mi.class_id = pi.class_id;

      mi.cam_id = pi.obj_id;
      mi.bbox.x = pi.x;
      mi.bbox.y = pi.y;
      mi.bbox.height = pi.height;
      mi.bbox.width = pi.width;

      // For CarMaker
      mi.d_y = pi.lateral;
      mi.d_z = pi.vertical;
      mi.width = pi.width;

      mi.track_mode = TrackMode::kUpdate;
      mi.match_flag = true;

      mi.unmatch_time = 0;

      is_match = true;

      // @Debug
      mi.radar_longitudinal = pi.longitudinal;
      mi.radar_lateral = pi.lateral;
      printf(
        "[Fusion %d]New matching has occurred. R:%d C:%d\n", mi.fusion_id, mi.radar_id, pi.obj_id);
      fflush(stdout);
    }
  }

  return is_match;
}

// 단일 개체에 대한 업데이트
bool PerceptMatching::Update(MatchInfo & mi, const PerceptInfo & pi)
{
  bool is_update = false;
  // Step 1: Checking case which estimation_id is same input_id
  if (mi.cam_id == pi.obj_id) {
    float match_box_area = (float)(mi.bbox.width * mi.bbox.height);
    float percept_box_area = (float)(pi.width * pi.height);
    // Obtain intersection area
    float min_x = (mi.bbox.x > pi.x) ? mi.bbox.x : pi.x;
    float min_y = (mi.bbox.y > pi.y) ? mi.bbox.y : pi.y;
    float max_x =
      (mi.bbox.x + mi.bbox.width < pi.x + pi.width) ? mi.bbox.x + mi.bbox.width : pi.x + pi.width;
    float max_y = (mi.bbox.y + mi.bbox.height < pi.y + pi.height) ? mi.bbox.y + mi.bbox.height
                                                                  : pi.y + pi.height;

    float width = (max_x - min_x + 1 > 0) ? max_x - min_x + 1 : 0;
    float height = (max_y - min_y + 1 > 0) ? max_y - min_y + 1 : 0;
    float overlap_area = width * height;

    float iou = (overlap_area) / (match_box_area + percept_box_area - overlap_area) * 100;

    if (iou > 30) {
      // For CarMaker
      mi.class_id = pi.class_id;

      mi.bbox.x = pi.x;
      mi.bbox.y = pi.y;
      mi.bbox.height = pi.height;
      mi.bbox.width = pi.width;

      // For CarMaker
      mi.d_x = pi.longitudinal;
      mi.d_y = pi.lateral;
      mi.d_z = pi.vertical;
      mi.width = pi.width;

      mi.track_mode = TrackMode::kUpdate;
      mi.match_flag = true;

      mi.unmatch_time = 0;

      is_update = true;

      // @Debug
      mi.radar_longitudinal = pi.longitudinal;
      mi.radar_lateral = pi.lateral;
    }
  }
  return is_update;
}
