#include "matcher/match_process.hpp"

#include <math.h>
#include <unistd.h>

#include <fstream>

#include "matcher/common.hpp"
#include "matcher/enum_class.hpp"
#include "matcher/matching/percept_matching.hpp"
#include "matcher/matching/radar_matching.hpp"

using namespace ebase::fusion::matcher;

MatchProcess::MatchProcess(const rclcpp::Logger & logger) : logger_(logger)
{
  RCLCPP_INFO(logger_, "Matchprocess initialization done");
}
int MatchProcess::InitializeMatchBuff()
{
  match_buff_.data.clear();
  // match_buff_.match_size = 0;

  if (match_buff_.size() != 0) {
    RCLCPP_ERROR(logger_, "The initialization step failed");
  }

  return match_buff_.size();
}

bool MatchProcess::Execute(std::unique_ptr<Matching> & exec)
{
  RCLCPP_DEBUG(logger_, "Start Match Process....Number of Input Data: %d", exec->GetDataCnt());
  exec->Attach(&fusion_id_);
  if (match_buff_.size() == 0 && exec->GetDataCnt() > 0) {
    // When there is no matched data, all input data is need to initialize and add match_buff_
    exec->InitializeData(match_buff_);

  } else if (match_buff_.size() != 0 && exec->GetDataCnt() > 0) {
    // Real data matching/update
    exec->MatchOrUpdate(match_buff_);
  } else {
    RCLCPP_ERROR(logger_, "There is no any input data...");
  }

  //  FusionAlgorithm state handling
  for (auto & match_info : match_buff_.data) {
    match_info->fusion->TryFusion(exec->GetType());
   /*  
    if (match_info->fusion->IsChanging()) {
      // match_info.fusion->CheckProgress();
      match_info->fusion->CheckProgress(exec->GetType());
    } else {
      match_info->fusion->TryChange(exec->GetType());
    } */
    DebugLogging(match_info);
  }

  return true;
}

int MatchProcess::UpdateMatchBufferWithEstimate(
  const erae_fusion_msgs::msg::FusionInfoArray & estimation_buff)
{
  for (auto & match_info : match_buff_.data) {
    for (const auto & estimation : estimation_buff.fusion_info) {
      if (match_info->fusion_id == estimation.fusion_id) {
        match_info->class_id = estimation.class_id;

        match_info->d_x = estimation.longitudinal_x;
        match_info->d_y = estimation.lateral_y;
        match_info->v_x = estimation.velocity_x;
        match_info->v_y = estimation.velocity_y;

        match_info->u = estimation.u;
        match_info->v = estimation.v;

        match_info->bbox = estimation.obj_box;
        match_info->track_mode = TrackMode::kUpdate;
        match_info->match_flag = false;
        match_info->sensor_type = static_cast<SensorType>(estimation.sensor_type);

        // For CarMaker
        match_info->d_z = estimation.vertical_z;
        match_info->width = estimation.width;
        match_info->a_x = estimation.a_x;
        match_info->confidence = estimation.confidence;
        match_info->motion_state = estimation.motion_state;
      }
    }
  }
  //RCLCPP_INFO(logger_, "Estimation data update done");
  return match_buff_.size();
}

void MatchProcess::CleanUpUnnecessaryData()
{
  for (int match_index = 0; match_index < (int)match_buff_.data.size(); match_index++) {
    if (match_buff_.data[match_index]->track_mode == TrackMode::kDelete) {
      vector<std::shared_ptr<MatchInfo>>::iterator iter = match_buff_.data.begin();
      fusion_id_.ResetId((*(iter + match_index))->fusion_id);
      match_buff_.data.erase(iter + match_index);
      match_index--;
    }
  }
}

void MatchProcess::DebugLogging(shared_ptr<MatchInfo> debug_info)
{
  string current_status("");
  /* if (debug_info->fusion->GetStatus() == FusionMode::kReady) {
    current_status = "Ready";
  } else if (debug_info->fusion->GetStatus() == FusionMode::kFusion) {
    current_status = "FusionAlgorithm";
  } else if (debug_info->fusion->GetStatus() == FusionMode::kUnFusion) {
    current_status = "UnFusion";
  } else {
    current_status = "None";
  } */
  string next_status("");
 /*  if (debug_info->fusion->GetNextStatus() == FusionMode::kReady) {
    next_status = "Ready";
  } else if (debug_info->fusion->GetNextStatus() == FusionMode::kFusion) {
    next_status = "FusionAlgorithm";
  } else if (debug_info->fusion->GetNextStatus() == FusionMode::kUnFusion) {
    next_status = "UnFusion";
  } else {
    next_status = "None";
  } */
  string type("");
  if (debug_info->sensor_type == SensorType::kCameraData) {
    type = "Camera Only";
  } else if (debug_info->sensor_type == SensorType::kRadarData) {
    type = "Radar Only";
  } else if (debug_info->sensor_type == SensorType::kFusionData) {
    type = "FusionAlgorithm";
  } else {
    type = "None";
  }

  RCLCPP_DEBUG(
    logger_, "fusion_id: %d\t sensor_type: %s, fusion_mode: %s, next_fusion_mode: %s",
    debug_info->fusion_id, type.c_str(), current_status.c_str(), next_status.c_str());
}
