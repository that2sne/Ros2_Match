#include "matcher/algorithm/radar_matching_algorithm.hpp"

#include "matcher/parameter/match_param_interface.hpp"
using ebase::fusion::matcher::RaderMatchingAlgorithm;

void RaderMatchingAlgorithm::Process(FusionData & fd, const RadarData & rd)
{
  // RCLCPP_DEBUG(logger_, "Start Match Process....Number of Input Data: %d", exec->GetDataCnt());
  if (fd.GetSize() == 0 && rd.GetSize() > 0) {
    // When there is no matched data, all input data is need to initialize and add match_buff_
    InitializeData(fd, rd);

  } else if (fd.GetSize() != 0 && rd.GetSize() > 0) {
    // Real data matching/update
    MatchOrUpdate(fd, rd);
  } else {
    // RCLCPP_ERROR(logger_, "There is no any input data...");
  }
}

void RaderMatchingAlgorithm::AddData(FusionData & fd, const RadarInfo & rd)
{
  // FusionData에 Radar 데이터를 추가 하는 행위
  fd += rd;
}

void RaderMatchingAlgorithm::InitializeData(FusionData & fd, const RadarData & rd)
{
  for (auto & i : rd) {
    AddData(fd, i);  // same  fd += rd;
  }
}

bool RaderMatchingAlgorithm::Match(MatchInfo & mi, const RadarInfo & ri, float & min_distance)
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
    }
  }
  return is_match;
}

// 단일 개체에 대한 업데이트
bool RaderMatchingAlgorithm::Update(MatchInfo & mi, const RadarInfo & ri)
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

void RaderMatchingAlgorithm::MatchOrUpdate(FusionData & fd, const RadarData & rd)
{
  int real_data_cnt = 0;
  // counting number of real data
  real_data_cnt = rd.GetSize();

  for (auto & i : fd) {
    i.match_flag = false;
  }

  // for (int real_index = 0; real_index < real_data_cnt; real_index++) {
  for (auto & ri : rd) {
    //  checking unmatch input data and any estimation
    float min_dist = 999999.999999;
    int prev_match_index = -1;
    bool match_flag = false;
    for (int match_index = 0; match_index < fd.GetSize(); match_index++)  // estimation buffer loop
    {
      bool is_match = false;

      // Step 2: Checking that fusion flag of estimation data is same input data
      // not using estimation data
      if (
        fd.data_[match_index].match_flag == false ||
        (fd.data_[match_index].cam_id == -1 || fd.data_[match_index].radar_id == -1))
      {
        // When input data type is the same estimation sensor_type, update estimation data with
        // input data
        if (fd.data_[match_index].sensor_type == GetType()) {
          is_match = Update(fd.data_[match_index], ri);
          if (match_flag == true) is_match = true;
        }
        // When input data type is not same estimation sensor_type, matching estimation data with
        // input data
        else
        {
          is_match = Match(fd.data_[match_index], ri, min_dist);
          // fd.data_[match_index].sensor_type = SensorType::kFusionData;
          if (match_flag == true && is_match == true && prev_match_index != -1) {
            if (fd.data_[prev_match_index].sensor_type == GetType())
              fd.data_[prev_match_index].track_mode = TrackMode::kDelete;
          }
          if (match_flag == true) is_match = true;
        }

        if (is_match == true) {
          if (prev_match_index != -1 && prev_match_index != match_index) {
            fd.data_[prev_match_index].match_flag = false;
            fd.data_[prev_match_index].unmatch_time = 10000;
            prev_match_index = match_index;
          }
          match_flag = true;

          if (min_dist == -1) break;
        }
      }
    }

    if (match_flag == false) {
      AddData(fd, ri);
    }
  }
}