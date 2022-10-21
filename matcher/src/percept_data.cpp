
#include "matcher/percept_data.hpp"

using ebase::fusion::matcher::PerceptData;
PerceptData::PerceptData(const erae_perception_msgs::msg::FCW & fcw_msg)
{
  for (std::size_t obj_index = 0; obj_index < fcw_msg.object_list.size(); obj_index++) {
    // Filtering Object
    if (
      fcw_msg.object_list[obj_index].class_obj < 0 || fcw_msg.object_list[obj_index].class_obj > 3)
      continue;

    PerceptInfo perception_info_;
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

    data_.push_back(perception_info_);
  }
}