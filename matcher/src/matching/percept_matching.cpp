#include "matcher/matching/percept_matching.hpp"

#include <cmath>

#include "matcher/fusion_algorithm/fusion_algorithm_factory.hpp"
#include "matcher/parameter/match_param_interface.hpp"

using namespace ebase::fusion::matcher;

PerceptMatching::~PerceptMatching() {}

PerceptMatching::PerceptMatching(PerceptInfoBuffer & perceptoin_info)
: perceptoin_info_buff_(perceptoin_info)
{
}

void PerceptMatching::AddData(MatchInfoPtrBuffer & match_info, int real_index)
{
  auto percept_info = perceptoin_info_buff_.data[real_index];
  auto temporary_match_info = std::make_shared<MatchInfo>();
  temporary_match_info->fusion_id = fusion_id_->GetId();

  // radar data
  temporary_match_info->radar_id = -1;
  temporary_match_info->v_x = 0;
  temporary_match_info->v_y = 0;
  temporary_match_info->u = 0;
  temporary_match_info->v = 0;

  // OD data
  temporary_match_info->cam_id = percept_info.obj_id;
  temporary_match_info->class_id = percept_info.class_id;
  temporary_match_info->bbox.x = percept_info.x;
  temporary_match_info->bbox.y = percept_info.y;
  temporary_match_info->bbox.width = percept_info.width;
  temporary_match_info->bbox.height = percept_info.height;

  // For CarMaker
  temporary_match_info->d_x = percept_info.longitudinal;
  temporary_match_info->d_y = percept_info.lateral;
  temporary_match_info->d_z = percept_info.vertical;
  temporary_match_info->width = percept_info.column;
  temporary_match_info->motion_state = 0;
  temporary_match_info->confidence = 0.0;

  // fusion Infomation
  temporary_match_info->sensor_type = SensorType::kCameraData;
  temporary_match_info->track_mode = TrackMode::kInit;
  temporary_match_info->match_flag = false;
  temporary_match_info->unmatch_time = 0;

  temporary_match_info->fusion =
    std::move(FusionAlgorithmFactory::CreateFusionAlgorithm(temporary_match_info));
  // temporary_match_info->fusion->Init();

  // @Debug
  temporary_match_info->radar_longitudinal = percept_info.longitudinal;
  temporary_match_info->radar_lateral = percept_info.lateral;

  match_info.data.push_back(temporary_match_info);
}

void PerceptMatching::InitializeData(MatchInfoPtrBuffer & match_info)
{
  for (int i = 0; i < perceptoin_info_buff_.size(); i++) {
    AddData(match_info, i);
  }
}

bool PerceptMatching::Update(MatchInfoPtrBuffer & match_buff_, int real_index, int match_index)
{
  PerceptInfo & percept_info = perceptoin_info_buff_.data[real_index];
  MatchInfo & match_info = *match_buff_.data[match_index];

  bool is_update = false;
  // Step 1: Checking case which estimation_id is same input_id
  if (match_info.cam_id == percept_info.obj_id) {
    float match_box_area = (float)(match_info.bbox.width * match_info.bbox.height);
    float percept_box_area = (float)(percept_info.width * percept_info.height);

    // Obtain intersection area
    float min_x = (match_info.bbox.x > percept_info.x) ? match_info.bbox.x : percept_info.x;
    float min_y = (match_info.bbox.y > percept_info.y) ? match_info.bbox.y : percept_info.y;
    float max_x = (match_info.bbox.x + match_info.bbox.width < percept_info.x + percept_info.width)
                    ? match_info.bbox.x + match_info.bbox.width
                    : percept_info.x + percept_info.width;
    float max_y =
      (match_info.bbox.y + match_info.bbox.height < percept_info.y + percept_info.height)
        ? match_info.bbox.y + match_info.bbox.height
        : percept_info.y + percept_info.height;

    float width = (max_x - min_x + 1 > 0) ? max_x - min_x + 1 : 0;
    float height = (max_y - min_y + 1 > 0) ? max_y - min_y + 1 : 0;
    float overlap_area = width * height;

    float iou = (overlap_area) / (match_box_area + percept_box_area - overlap_area) * 100;
    if (iou > 30) {
      // For CarMaker
      match_info.class_id = percept_info.class_id;

      match_info.bbox.x = percept_info.x;
      match_info.bbox.y = percept_info.y;
      match_info.bbox.height = percept_info.height;
      match_info.bbox.width = percept_info.width;

      // For CarMaker
      match_info.d_x = percept_info.longitudinal;
      match_info.d_y = percept_info.lateral;
      match_info.d_z = percept_info.vertical;
      match_info.width = percept_info.width;

      match_info.track_mode = TrackMode::kUpdate;
      match_info.match_flag = true;

      match_info.unmatch_time = 0;

      is_update = true;

      // @Debug
      match_info.radar_longitudinal = percept_info.longitudinal;
      match_info.radar_lateral = percept_info.lateral;
    }
  }
  return is_update;
}

bool PerceptMatching::Match(
  MatchInfoPtrBuffer & match_buff_, int real_index, int match_index, float & min_distance)
{
  PerceptInfo & percept_info = perceptoin_info_buff_.data[real_index];
  MatchInfo & match_info = *match_buff_.data[match_index];

  bool is_match = false;
  // Step 1: Matching ROI 계산 - matching할 radar point를 탐색하는 영역 제한
  //          * base_num^(-distance)
  int limit_region_x =
    (int)(percept_info.width / (float)2 + 0.5) +
    (int)(MatchParamMaxSearchingRange() * pow(MatchParamBaseNum(), (-(match_info.d_x / 100))));
  int limit_region_y =
    (int)(percept_info.height / (float)2 + 0.5) +
    (int)(MatchParamMaxSearchingRange() * pow(MatchParamBaseNum(), (-(match_info.d_x / 100))));

  // Step 2: Check radar point is located inside Matching ROI
  int od_center_x = (int)percept_info.x + (int)(percept_info.width / (float)2 + 0.5);
  int od_center_y = (int)percept_info.y + (int)(percept_info.height / (float)2 + 0.5);

  if (match_info.cam_id == percept_info.obj_id) {
    min_distance = -1;

    // For CarMaker
    match_info.class_id = percept_info.class_id;

    match_info.cam_id = percept_info.obj_id;
    match_info.bbox.x = percept_info.x;
    match_info.bbox.y = percept_info.y;
    match_info.bbox.height = percept_info.height;
    match_info.bbox.width = percept_info.width;

    match_info.u = match_info.u;
    match_info.v = match_info.v;

    // For CarMaker
    match_info.d_y = percept_info.lateral;
    match_info.d_z = percept_info.vertical;
    match_info.width = percept_info.width;

    match_info.track_mode = TrackMode::kUpdate;
    match_info.match_flag = true;

    match_info.unmatch_time = 0;

    is_match = true;

    // @Debug
    match_info.radar_longitudinal = percept_info.longitudinal;
    match_info.radar_lateral = percept_info.lateral;
  } else if (
    abs(od_center_x - match_info.u) <= limit_region_x &&
    abs(od_center_y - match_info.v) <= limit_region_y)
  {
    float distance = pow((od_center_x - match_info.u), 2) + pow((od_center_y - match_info.v), 2);
    distance = sqrt(distance);

    float threshold_longitudinal =
      match_info.v_x == 0 ? 3.0 : match_info.v_x * PeriodSensor::CAMERA_PERIOD;
    threshold_longitudinal =
      match_info.sensor_type == SensorType::kFusionData ? threshold_longitudinal : 200;
    float diff_longitudinal = abs(match_info.d_x - percept_info.longitudinal);

    // Step 3. Find the closest radar point to Bbox center
    if (distance < min_distance && diff_longitudinal < threshold_longitudinal) {
      min_distance = distance;
      // For CarMaker
      match_info.class_id = percept_info.class_id;

      match_info.cam_id = percept_info.obj_id;
      match_info.bbox.x = percept_info.x;
      match_info.bbox.y = percept_info.y;
      match_info.bbox.height = percept_info.height;
      match_info.bbox.width = percept_info.width;

      match_info.u = match_info.u;
      match_info.v = match_info.v;

      // For CarMaker
      match_info.d_y = percept_info.lateral;
      match_info.d_z = percept_info.vertical;
      match_info.width = percept_info.width;

      match_info.track_mode = TrackMode::kUpdate;
      match_info.match_flag = true;

      match_info.unmatch_time = 0;

      is_match = true;

      // @Debug
      match_info.radar_longitudinal = percept_info.longitudinal;
      match_info.radar_lateral = percept_info.lateral;
    }
  }

  return is_match;
}
