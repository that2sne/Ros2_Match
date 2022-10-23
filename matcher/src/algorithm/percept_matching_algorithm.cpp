#include "matcher/algorithm/percept_matching_algorithm.hpp"

#include "matcher/parameter/match_param_interface.hpp"
using ebase::fusion::matcher::PerceptMatchingAlgorithm;

void PerceptMatchingAlgorithm::Process(FusionData & fd, const PerceptData & pd)
{
  if (fd.GetSize() == 0 && pd.GetSize() > 0) {
    InitializeData(fd, pd);

  } else if (fd.GetSize() != 0 && pd.GetSize() > 0) {
    MatchOrUpdate(fd, pd);
  } else {
    // RCLCPP_ERROR(logger_, "There is no any input data...");
  }

  fd.UpdateStatus(GetType());
}

void PerceptMatchingAlgorithm::AddData(FusionData & fd, const PerceptInfo & pd)
{
  //데이터 추가.
  fd += pd;
}

void PerceptMatchingAlgorithm::InitializeData(FusionData & fd, const PerceptData & pd)
{
  for (auto & i : pd) {
    AddData(fd, i);  // same  fd += pd;
  }
}

bool PerceptMatchingAlgorithm::Match(MatchInfo & mi, const PerceptInfo & pi, float & min_distance)
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
    }
  }

  return is_match;
}

// 단일 개체에 대한 업데이트
bool PerceptMatchingAlgorithm::Update(MatchInfo & mi, const PerceptInfo & pi)
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

void PerceptMatchingAlgorithm::MatchOrUpdate(FusionData & fd, const PerceptData & pd)
{
  int real_data_cnt = 0;
  // counting number of real data
  real_data_cnt = pd.GetSize();

  for (auto & i : fd) {
    i.match_flag = false;
  }

  // for (int real_index = 0; real_index < real_data_cnt; real_index++) {
  for (auto & ri : pd) {
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