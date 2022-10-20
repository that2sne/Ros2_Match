#include "matcher/matching/matching.hpp"

using namespace ebase::fusion::matcher;

Matching::~Matching() {}
void Matching::Attach(FusionIdManager * id) { fusion_id_ = id; }
void Matching::MatchOrUpdate(MatchInfoPtrBuffer & match_buff_)
{
  int real_data_cnt = 0;
  // counting number of real data
  real_data_cnt = GetDataCnt();
  //(kind_of_input_data == kCameraData) ? percept_buff_.data_cnt : radar_buff_.data_cnt;

  for (int match_index = 0; match_index < match_buff_.size(); match_index++) {
    match_buff_.data[match_index]->match_flag = false;
  }

  for (int real_index = 0; real_index < real_data_cnt; real_index++) {
    // checking unmatch input data and any estimation
    float min_dist = 999999.999999;
    int prev_match_index = -1;
    bool match_flag = false;
    for (int match_index = 0; match_index < match_buff_.size();
         match_index++)  // estimation buffer loop
    {
      bool is_match = false;

      // Step 2: Checking that fusion flag of estimation data is same input data
      // not using estimation data
      if (
        match_buff_.data[match_index]->match_flag == false ||
        (match_buff_.data[match_index]->cam_id == -1 ||
         match_buff_.data[match_index]->radar_id == -1))
      {
        // When input data type is the same estimation sensor_type, update estimation data with
        // input data
        if (match_buff_.data[match_index]->sensor_type == GetType()) {
          is_match = Update(match_buff_, real_index, match_index);
          if (match_flag == true) is_match = true;
        }
        // When input data type is not same estimation sensor_type, matching estimation data with
        // input data
        else
        {
          is_match = Match(match_buff_, real_index, match_index, min_dist);
          //match_buff_.data[match_index]->sensor_type = SensorType::kFusionData;
          if (match_flag == true && is_match == true && prev_match_index != -1) {
            if (match_buff_.data[prev_match_index]->sensor_type == GetType())
              match_buff_.data[prev_match_index]->track_mode = TrackMode::kDelete;
          }
          if (match_flag == true) is_match = true;
        }

        if (is_match == true) {
          if (prev_match_index != -1 && prev_match_index != match_index) {
            match_buff_.data[prev_match_index]->match_flag = false;
            match_buff_.data[prev_match_index]->unmatch_time = 10000;
            prev_match_index = match_index;
          }
          match_flag = true;

          if (min_dist == -1) break;
        }
      }
    }

    if (match_flag == false) {
      AddData(match_buff_, real_index);
    }
  }
}