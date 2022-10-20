#include "matcher/match_node.hpp"

#include <time.h>

#include <opencv2/opencv.hpp>

#include "matcher/common.hpp"
#include "matcher/fusion_algorithm/fusion_algorithm_factory.hpp"
#include "matcher/matching/percept_matching.hpp"
#include "matcher/matching/radar_matching.hpp"
#include "matcher/parameter/match_parameter.hpp"

#include "matcher/radar_matching_command.hpp"
using namespace ebase::fusion::matcher;
using namespace cv;

Matcher::Matcher() : EBaseNode("matcher", erae_diag_msgs::msg::ModuleSts::MODULE_FUSION_MATCH)
{
  // publish to rviz
  publisher_draw_info = this->create_publisher<erae_fusion_msgs::msg::DrawInfo>("match", 10);
  publisher_matchedInfo =
    this->create_publisher<erae_fusion_msgs::msg::FusionInfoArray>("match_info_track_topic", 10);
  // @Debug
  // publisher_debug =
  //   this->create_publisher<erae_fusion_msgs::msg::MatchDebugArray>("match_debug", 10);

  // from perception
  subscriber_bbox = this->create_subscription<erae_perception_msgs::msg::FCW>(
    "object_detection", 10, std::bind(&Matcher::SubscribeBboxCallback, this, _1));

  // from radar
  subscriber_radar = this->create_subscription<erae_sensor_msgs::msg::MrrInfoArray>(
    "mrr", 10, std::bind(&Matcher::SubscribeRadarCallback, this, _1));

  // from tracking node
  subscriber_estiMatchedInfo = this->create_subscription<erae_fusion_msgs::msg::FusionInfoArray>(
    "estimation", 10, std::bind(&Matcher::SubscribeEstimateCallback, this, _1));

  // For Demo: get vehicle information
  subscriber_vehicle = this->create_subscription<erae_vehicle_msgs::msg::VehicleInfo>(
    "vehicle_info", 10, std::bind(&Matcher::SubscribeVehicleInfo, this, _1));

  /// @brief CmInfo message is subscribed from CM_bridge.
  subscriber_scenario_status = this->create_subscription<std_msgs::msg::String>(
    "sim_status", 10, std::bind(&Matcher::SubscribeFromCmBridgeScenarioStatusCallback, this, _1));

  matching_process_ = std::make_unique<MatchProcess>(this->get_logger());
  // @jm: 상대속도 기반 혹은 자차 속도 기반 fusion 상태변이 알고리즘 선택
  // FusionAlgorithmFactory::getInstance()->SetAlgorithm(
  //   FusionAlgorithmFactory::FusionAlgorithmType::kRelativeSpeedFusion);
  FusionAlgorithmFactory::getInstance()->SetAlgorithm(
    FusionAlgorithmFactory::FusionAlgorithmType::kSimpleFusion);

  // Thread initialization
  thread_handle_ = std::thread(&Matcher::WorkerThread, this);
}

void Matcher::SubscribeBboxCallback(const erae_perception_msgs::msg::FCW::SharedPtr msg)
{
  PerceptInfoBuffer percept_buff_;
  CovertPerceptMsg2Struct(*msg, percept_buff_);
  // ConvertPerceptionMsg2DrawMsg(*msg);

  std::unique_ptr<Matching> exec = std::make_unique<PerceptMatching>(percept_buff_);
  PushQueue(exec);
}

void Matcher::ConvertPerceptionMsg2DrawMsg(erae_perception_msgs::msg::FCW & fcw_msg)
{
  // Input camera data to draw draw_info_msg
  draw_info_msg.image_path = fcw_msg.image_path;
  draw_info_msg.input_perception_info.clear();

  for (int i = 0; i < fcw_msg.target_object_number; i++) {
    if (fcw_msg.object_list[i].class_obj < 0 || fcw_msg.object_list[i].class_obj > 3) {
      continue;
    }
    erae_fusion_msgs::msg::FusionInfo perception_info;

    perception_info.class_id = fcw_msg.object_list[i].class_obj;
    perception_info.fusion_id = fcw_msg.object_list[i].id;

    perception_info.obj_box.x = fcw_msg.tracking_list[i].obj_box.x;
    perception_info.obj_box.y = fcw_msg.tracking_list[i].obj_box.y;
    perception_info.obj_box.width = fcw_msg.tracking_list[i].obj_box.width;
    perception_info.obj_box.height = fcw_msg.tracking_list[i].obj_box.height;

    // For CarMaker
    // YJ
    perception_info.longitudinal_x = fcw_msg.object_list[i].point.x;
    perception_info.lateral_y = fcw_msg.object_list[i].point.y;
    perception_info.vertical_z = fcw_msg.object_list[i].point.z;
    perception_info.width = fcw_msg.object_list[i].width;
    perception_info.class_id = fcw_msg.object_list[i].class_obj;

    draw_info_msg.input_perception_info.push_back(perception_info);
  }
}

void Matcher::SubscribeRadarCallback(const erae_sensor_msgs::msg::MrrInfoArray::SharedPtr msg)
{
  //RadarInfoBuffer radar_buff_;
  //ConvertRadarMsg2Struct(*msg, radar_buff_);
  // ConvertRadarMsg2DrawMsg(*msg);

  //std::unique_ptr<Matching> exec = std::make_unique<RadarMatching>(radar_buff_);
  //PushQueue(exec);
  PushQueue(RadarMathcingCommand(RadarData(*msg)));
  //RadarMathcingCommand ff(RadarData(*msg));
}

void Matcher::ConvertRadarMsg2DrawMsg(erae_sensor_msgs::msg::MrrInfoArray & radar_msg)
{
  // Input radar data to draw draw_info_msg
  draw_info_msg.input_radar_info.clear();
  for (int i = 0; i < (int)radar_msg.mrrinfo_array.size(); i++) {
    erae_fusion_msgs::msg::FusionInfo radar_info;
    // Filtering Object
    if (radar_msg.mrrinfo_array[i].motion_state < 2 || radar_msg.mrrinfo_array[i].motion_state > 4)
    {
      continue;
    }

    radar_info.fusion_id = radar_msg.mrrinfo_array[i].id;
    radar_info.longitudinal_x = radar_msg.mrrinfo_array[i].range_x;
    radar_info.lateral_y = radar_msg.mrrinfo_array[i].range_y;
    radar_info.u = radar_msg.mrrinfo_array[i].u;
    radar_info.v = radar_msg.mrrinfo_array[i].v;
    radar_info.velocity_x = radar_msg.mrrinfo_array[i].speed_x;

    radar_info.velocity_y = radar_msg.mrrinfo_array[i].speed_y;

    // CarMaker
    // YJ YUN
    radar_info.vertical_z = radar_msg.mrrinfo_array[i].height;
    radar_info.width = radar_msg.mrrinfo_array[i].width;
    radar_info.a_x = radar_msg.mrrinfo_array[i].a_x;
    radar_info.confidence = radar_msg.mrrinfo_array[i].confidence;
    radar_info.motion_state = radar_msg.mrrinfo_array[i].motion_state;

    draw_info_msg.input_radar_info.push_back(radar_info);
  }
}

void Matcher::SubscribeEstimateCallback(const erae_fusion_msgs::msg::FusionInfoArray::SharedPtr msg)
{
  estimation_buff_m_.lock();
  estimation_buff_ = *msg;
  estimation_buff_m_.unlock();
}

void Matcher::SubscribeVehicleInfo(const erae_vehicle_msgs::msg::VehicleInfo::SharedPtr msg)
{
  erae_vehicle_msgs::msg::VehicleInfo vehicle_info = *msg;

  float ego_speed = vehicle_info.veh_spd;
}

bool Matcher::SubscribeFromCmBridgeScenarioStatusCallback(
  const std_msgs::msg::String::SharedPtr msg)
{
  scenario_status_msg = *msg;

  if (scenario_status_msg.data == "start") {
    int buffer_size = matching_process_->InitializeMatchBuff();

    if (buffer_size != 0) return false;
  }

  return true;
}

void Matcher::PushQueue(Command && exec)
{
  std::lock(exec_queue_m_, cv_m_);
  std::lock_guard<std::mutex> lk1(exec_queue_m_, std::adopt_lock);
  std::lock_guard<std::mutex> lk2(cv_m_, std::adopt_lock);
  exec_queue_.push_back(std::move(exec));
  cv_.notify_all();
}

bool Matcher::PopQueue(Command& cmd)
{
  std::lock_guard<std::mutex> lg(exec_queue_m_);
  if (!exec_queue_.empty()) {
    return false;
  }
  cmd = std::move(exec_queue_.front());
  exec_queue_.pop_front();
  
  return true;;
}

void Matcher::WorkerThread()
{
  while (1) {
    Command obj;
    {
      std::unique_lock<std::mutex> lk(cv_m_);
      bool b = PopQueue(obj);
      // std::this_thread::sleep_for(50ms);
      if (!b) {
        cv_.wait(lk, [&] { return !exec_queue_.empty(); });
        continue;
      }
    }

    estimation_buff_m_.lock();
    matching_process_->UpdateMatchBufferWithEstimate(estimation_buff_);
    estimation_buff_m_.unlock();

    obj.exectue();
    //matching_process_->Execute(obj);

    ConvertFusionStruct2Msg(matching_process_->GetMatchBuf(), matched_msg);

    publisher_draw_info->publish(draw_info_msg);

    draw_info_msg.input_perception_info.clear();
    draw_info_msg.input_radar_info.clear();
    draw_info_msg.match_info_array.clear();

    // @Debug
    // publisher_debug->publish(debug_msg);
    // debug_msg.match_info.clear();

    matching_process_->CleanUpUnnecessaryData();

    // fflush(stdout);
    //  c_matching_process.match_buff_.match_size =
    //  c_matching_process.match_buff_.match_infoes.size();
  }
}

bool Matcher::ConvertRadarMsg2Struct(
  erae_sensor_msgs::msg::MrrInfoArray & radar_msg, RadarInfoBuffer & radar_buff_)
{
  // radar_buff_.data_cnt = radar_msg.mrrinfo_array.size();
  // radar_buff_.radar_infoes.clear();

  /*  if (radar_buff_.timestamp < 0 || radar_msg.mrrinfo_array.size() <= 0) {
     return false;
   }
  */
  for (int i = 0; i < (int)radar_msg.mrrinfo_array.size(); i++) {
    // Filtering Object
    if (radar_msg.mrrinfo_array[i].motion_state < 2 || radar_msg.mrrinfo_array[i].motion_state > 4)
    {
      continue;
    }

    RadarInfo radar_info;

    radar_info.target_id = radar_msg.mrrinfo_array[i].id;
    radar_info.longitudinal_x = radar_msg.mrrinfo_array[i].range_x;
    radar_info.lateral_y = radar_msg.mrrinfo_array[i].range_y;
    radar_info.u = radar_msg.mrrinfo_array[i].u;
    radar_info.v = radar_msg.mrrinfo_array[i].v;
    radar_info.v_x = radar_msg.mrrinfo_array[i].speed_x;
    radar_info.v_y = radar_msg.mrrinfo_array[i].speed_y;

    // CarMaker
    // YJ YUN
    radar_info.vertical_z = radar_msg.mrrinfo_array[i].height;
    radar_info.width = radar_msg.mrrinfo_array[i].width;
    radar_info.a_x = radar_msg.mrrinfo_array[i].a_x;
    radar_info.confidence = radar_msg.mrrinfo_array[i].confidence;
    radar_info.motion_state = radar_msg.mrrinfo_array[i].motion_state;

    radar_buff_.data.push_back(radar_info);
  }

  // radar_buff_.data_cnt = radar_buff_.radar_infoes.size();
  return true;
}

bool Matcher::CovertPerceptMsg2Struct(
  erae_perception_msgs::msg::FCW & fcw_msg, PerceptInfoBuffer & percept_buff_)
{
  // percept_buff_.data_cnt = fcw_msg.object_list.size();
  // percept_buff_.data.clear();

  /* if (fcw_msg.timestamp < 0 || fcw_msg.object_list.size() <= 0) {
    return false;
  } */

  PerceptInfo perception_info_;
  for (std::size_t obj_index = 0; obj_index < fcw_msg.object_list.size(); obj_index++) {
    // Filtering Object
    if (
      fcw_msg.object_list[obj_index].class_obj < 0 || fcw_msg.object_list[obj_index].class_obj > 3)
      continue;

    perception_info_.class_id = fcw_msg.object_list[obj_index].class_obj;
    perception_info_.obj_id = fcw_msg.object_list[obj_index].id;
    perception_info_.x = fcw_msg.tracking_list[obj_index].obj_box.x;
    perception_info_.y = fcw_msg.tracking_list[obj_index].obj_box.y;
    perception_info_.width = fcw_msg.tracking_list[obj_index].obj_box.width;
    perception_info_.height = fcw_msg.tracking_list[obj_index].obj_box.height;

    // YJ
    // For CarMaker
    perception_info_.longitudinal = fcw_msg.object_list[obj_index].point.x;
    perception_info_.lateral = fcw_msg.object_list[obj_index].point.y;
    perception_info_.vertical = fcw_msg.object_list[obj_index].point.z;
    perception_info_.column = fcw_msg.object_list[obj_index].width;
    perception_info_.class_id = fcw_msg.object_list[obj_index].class_obj;

    percept_buff_.data.push_back(perception_info_);
  }

  // percept_buff_.data_cnt = percept_buff_.obj_infoes.size();

  return true;
}

bool Matcher::ConvertFusionStruct2Msg(
  MatchInfoPtrBuffer & match_buff_, erae_fusion_msgs::msg::FusionInfoArray & match_msg)
{
  match_msg.fusion_size = match_buff_.size();
  match_msg.fusion_info.clear();

  if (match_buff_.size() <= 0) {
    return false;
  }

  erae_fusion_msgs::msg::FusionInfo match_data;
  // @Debug
  // erae_fusion_msgs::msg::MatchInOutDebug debug_data;
  for (int match_index = 0; match_index < match_msg.fusion_size; match_index++) {
    match_data.fusion_id = match_buff_.data[match_index]->fusion_id;
    match_data.class_id = match_buff_.data[match_index]->class_id;

    match_data.longitudinal_x = match_buff_.data[match_index]->d_x;
    match_data.lateral_y = match_buff_.data[match_index]->d_y;

    match_data.velocity_x = match_buff_.data[match_index]->v_x;
    match_data.velocity_y = match_buff_.data[match_index]->v_y;

    match_data.u = match_buff_.data[match_index]->u;
    match_data.v = match_buff_.data[match_index]->v;

    match_data.obj_box = match_buff_.data[match_index]->bbox;

    match_data.track_mode = static_cast<int>(match_buff_.data[match_index]->track_mode);
    match_data.sensor_type = static_cast<int>(match_buff_.data[match_index]->sensor_type);

    // For CarMaker
    // For CarMaker
    match_data.a_x = match_buff_.data[match_index]->a_x;
    match_data.vertical_z = match_buff_.data[match_index]->d_z;
    match_data.width = match_buff_.data[match_index]->width;
    match_data.confidence = match_buff_.data[match_index]->confidence;
    match_data.motion_state = match_buff_.data[match_index]->motion_state;
    //match_data.fusion_mode = static_cast<int>(match_buff_.data[match_index]->fusion->GetStatus());
    match_data.fusion_mode = 3;

    match_msg.fusion_info.push_back(match_data);
    draw_info_msg.match_info_array.push_back(match_data);

    // @Debug
    /*debug_data.fusion_id = match_buff_.data[match_index].fusion_id;
    debug_data.cam_id = match_buff_.data[match_index].cam_id;
    debug_data.radar_id = match_buff_.data[match_index].radar_id;
    debug_data.cam_longitudinal = match_buff_.data[match_index].cam_longitudinal;
    debug_data.cam_lateral = match_buff_.data[match_index].cam_lateral;
    debug_data.radar_longitudinal = match_buff_.data[match_index].radar_longitudinal;
    debug_data.radar_lateral = match_buff_.data[match_index].radar_lateral;
    debug_data.velocity_x = match_buff_.data[match_index].v_x;
    debug_data.velocity_y = match_buff_.data[match_index].v_y;
    debug_data.track_mode = static_cast<int>(match_buff_.data[match_index].track_mode);
    debug_data.sensor_type = static_cast<int>(match_buff_.data[match_index].sensor_type);
    debug_data.fusion_mode = static_cast<int>(match_buff_.data[match_index].fusion->GetStatus());
    debug_data.next_mode = static_cast<int>(match_buff_.data[match_index].fusion->GetNextStatus());
    //debug_data.duration_ready;
    //debug_data.duration_fusion;
    //debug_data.duration_unfusion;

    int is_update = false;
    for(auto & debug_info : debug_msg.match_info) {
      if(debug_info.fusion_id == debug_data.fusion_id) {
        debug_info = debug_data;
        is_update = true;
      }
    }
    if(is_update == false && debug_msg.match_info.size()<64)
      debug_msg.match_info.push_back(debug_data);*/
  }

  return true;
}