#include "matcher/fusion_data.hpp"

#include "matcher/enum_class.hpp"
#include "matcher/fusion_algorithm/fusion_algorithm_factory.hpp"
using ebase::fusion::matcher::FusionData;

FusionData::FusionData()
{
}
FusionData & FusionData::operator+=(const RadarInfo & rd)
{
  // FusionData에 Radar 데이터를 추가 하는 행위
  MatchInfo temporary_match_info;
  temporary_match_info.fusion_id = fusion_id_.GetId();

  // radar data
  temporary_match_info.radar_id = rd.target_id;
  temporary_match_info.d_x = rd.longitudinal_x;
  temporary_match_info.d_y = rd.lateral_y;
  temporary_match_info.v_x = rd.v_x;
  temporary_match_info.v_y = rd.v_y;
  temporary_match_info.u = rd.u;
  temporary_match_info.v = rd.v;

  // For CarMaker
  temporary_match_info.d_z = rd.vertical_z;
  temporary_match_info.width = rd.width;
  temporary_match_info.a_x = rd.a_x;

  temporary_match_info.confidence = rd.confidence;
  temporary_match_info.motion_state = rd.motion_state;

  // OD data
  temporary_match_info.cam_id = -1;
  temporary_match_info.class_id = 0;
  temporary_match_info.bbox.x = 0;
  temporary_match_info.bbox.y = 0;
  temporary_match_info.bbox.width = 0;
  temporary_match_info.bbox.height = 0;

  // fusion Infomation
  temporary_match_info.sensor_type = SensorType::kRadarData;
  temporary_match_info.track_mode = TrackMode::kInit;
  temporary_match_info.match_flag = false;
  temporary_match_info.unmatch_time = 0;

  // temporary_match_info.fusion =
  // FusionAlgorithmFactory::CreateFusionAlgorithm(temporary_match_info);
  //  temporary_match_info.fusion->Init();

  temporary_match_info.radar_longitudinal = rd.longitudinal_x;
  temporary_match_info.radar_lateral = rd.lateral_y;

  data_.push_back(temporary_match_info);
  return *this;
}

FusionData & FusionData::operator+=(const PerceptInfo & rd)
{
  MatchInfo temporary_match_info;
  temporary_match_info.fusion_id = fusion_id_.GetId();

  // radar data
  temporary_match_info.radar_id = -1;
  temporary_match_info.v_x = 0;
  temporary_match_info.v_y = 0;
  temporary_match_info.u = 0;
  temporary_match_info.v = 0;

  // OD data
  temporary_match_info.cam_id = rd.obj_id;
  temporary_match_info.class_id = rd.class_id;
  temporary_match_info.bbox.x = rd.x;
  temporary_match_info.bbox.y = rd.y;
  temporary_match_info.bbox.width = rd.width;
  temporary_match_info.bbox.height = rd.height;

  // For CarMaker
  temporary_match_info.d_x = rd.longitudinal;
  temporary_match_info.d_y = rd.lateral;
  temporary_match_info.d_z = rd.vertical;
  temporary_match_info.width = rd.column;
  temporary_match_info.motion_state = 0;
  temporary_match_info.confidence = 0.0;

  // fusion Infomation
  temporary_match_info.sensor_type = SensorType::kCameraData;
  temporary_match_info.track_mode = TrackMode::kInit;
  temporary_match_info.match_flag = false;
  temporary_match_info.unmatch_time = 0;

   // temporary_match_info.fusion =
  // std::move(FusionAlgorithmFactory::CreateFusionAlgorithm(temporary_match_info));
  // temporary_match_info.fusion->Init();

  // @Debug
  temporary_match_info.radar_longitudinal = rd.longitudinal;
  temporary_match_info.radar_lateral = rd.lateral;

  data_.push_back(temporary_match_info);
  return *this;
}
FusionData & FusionData::operator+=(const erae_fusion_msgs::msg::FusionInfoArray & estimation_buff)
{
  for (auto & match_info : data_) {
    for (const auto & estimation : estimation_buff.fusion_info) {
      if (match_info.fusion_id == estimation.fusion_id) {
        match_info.class_id = estimation.class_id;

        match_info.d_x = estimation.longitudinal_x;
        match_info.d_y = estimation.lateral_y;
        match_info.v_x = estimation.velocity_x;
        match_info.v_y = estimation.velocity_y;

        match_info.u = estimation.u;
        match_info.v = estimation.v;

        match_info.bbox = estimation.obj_box;
        match_info.track_mode = TrackMode::kUpdate;
        match_info.match_flag = false;
        match_info.sensor_type = static_cast<SensorType>(estimation.sensor_type);

        // For CarMaker
        match_info.d_z = estimation.vertical_z;
        match_info.width = estimation.width;
        match_info.a_x = estimation.a_x;
        match_info.confidence = estimation.confidence;
        match_info.motion_state = estimation.motion_state;
      }
    }
  }
  return *this;
}

void FusionData::CleanUpUnnecessaryData()
{
  for (int match_index = 0; match_index < (int)data_.size(); match_index++) {
    if (data_[match_index].track_mode == TrackMode::kDelete) {
      vector<MatchInfo>::iterator iter = data_.begin();
      FusionAlgorithmFactory::GetInstance()->Delete((*(iter + match_index)).fusion_id);
      fusion_id_.ResetId((*(iter + match_index)).fusion_id);

      data_.erase(iter + match_index);
      match_index--;
    }
  }
}
