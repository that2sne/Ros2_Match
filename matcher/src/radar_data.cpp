#include "matcher/radar_data.hpp"

using ebase::fusion::matcher::RadarData;
RadarData::RadarData(const erae_sensor_msgs::msg::MrrInfoArray & radar_msg)
{
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

    data_.push_back(radar_info);
  }
}