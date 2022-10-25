#ifndef FUSION__MATCHER_FUSION_DATA_HPP_
#define FUSION__MATCHER_FUSION_DATA_HPP_

#include "common.hpp"
#include "erae_fusion_msgs/msg/draw_info.hpp"
#include "erae_fusion_msgs/msg/fusion_info_array.hpp"
#include "fusion_algorithm/fusion_algorithm.hpp"
#include "fusion_id_manager.hpp"

#include <vector>
namespace ebase
{
namespace fusion
{
namespace matcher
{
using ebase::fusion::matcher::FusionIdManager;
using ebase::fusion::matcher::MatchInfo;
using std::vector;
class FusionData
{
public:
  vector<struct MatchInfo> data_;
  FusionIdManager fusion_id_;

public:
  FusionData();
  size_t GetSize() { return data_.size(); }
  FusionData & operator+=(const RadarInfo & rd);    // 신규 추가
  FusionData & operator+=(const PerceptInfo & rd);  // 신규 추가
  FusionData & operator+=(const erae_fusion_msgs::msg::FusionInfoArray & estimation_buff);
  operator erae_fusion_msgs::msg::DrawInfo() const
  {
    auto msg = erae_fusion_msgs::msg::DrawInfo();
    // auto msg = erae_fusion_msgs::msg::FusionInfoArray();
    erae_fusion_msgs::msg::FusionInfo match_data;
    for (size_t match_index = 0; match_index < data_.size(); match_index++) {
      match_data.fusion_id = data_[match_index].fusion_id;
      match_data.class_id = data_[match_index].class_id;

      match_data.longitudinal_x = data_[match_index].d_x;
      match_data.lateral_y = data_[match_index].d_y;

      match_data.velocity_x = data_[match_index].v_x;
      match_data.velocity_y = data_[match_index].v_y;

      match_data.u = data_[match_index].u;
      match_data.v = data_[match_index].v;

      match_data.obj_box = data_[match_index].bbox;

      match_data.track_mode = static_cast<int>(data_[match_index].track_mode);
      match_data.sensor_type = static_cast<int>(data_[match_index].sensor_type);

      // For CarMaker
      // For CarMaker
      match_data.a_x = data_[match_index].a_x;
      match_data.vertical_z = data_[match_index].d_z;
      match_data.width = data_[match_index].width;
      match_data.confidence = data_[match_index].confidence;
      match_data.motion_state = data_[match_index].motion_state;
      // match_data.fusion_mode =
      // static_cast<int>(data_[match_index].fusion->GetStatus());
      match_data.fusion_mode = 3;

      // match_msg.fusion_info.push_back(match_data);
      msg.match_info_array.push_back(match_data);
    }
    return msg;
  }

  void CleanUpUnnecessaryData();
  decltype(data_.begin()) begin() { return data_.begin(); }
  decltype(data_.end()) end() { return data_.end(); }

  void Clear() { data_.clear(); }
};
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_FUSION_DATA_HPP_