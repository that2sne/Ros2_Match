#ifndef FUSION__MATCHER_COMMON_HPP_
#define FUSION__MATCHER_COMMON_HPP_

#include <vector>

#include "enum_class.hpp"
#include "erae_std_msgs/msg/birdeye_view.hpp"
#include "erae_std_msgs/msg/bounding_box2_d.hpp"
#include "erae_std_msgs/msg/tracking_object.hpp"
//#include "matcher/fusion_algorithm/fusion_algorithm.hpp"
namespace ebase
{
namespace fusion
{
namespace matcher
{
class FusionAlgorithm;
struct PerceptInfo {
  /**
   * @var int class_id
   * Type of detected object: car(0), bus(1), truck(2), pedestroan(10),...
   * @var int obj_id
   * Object unique id value
   * @var double x
   * x coordinate of object bbox's left-top point
   * @var double y
   * y coordinate of object bbox's left-top point
   * @var double width
   * width of object bbox(unit: pixel)
   * @var double height
   * height of object bbox(unit: pixel)
   * @var double lateral
   * Lateral distance between ego-vehicle and target object (unit: meter)
   * @var double longitudinal
   * Longitudinal distance between ego-vehicle and target object (unit: meter)
   * @var double vertical
   * Height from the road to top of target object (unit: meter)
   */
  unsigned char class_id;
  int obj_id;
  double x;
  double y;
  double width;
  double height;
  double lateral;
  double longitudinal;
  double vertical;
  double column;
};
/**
 * @struct PerceptInfoBuffer
 * @brief struct included information recived from perceptrion node
 * @details This structure stores timestamp and detected object information
 * @author jimin cho
 * @date 2022.05.19
 * @version 0.1
 */

/**
 * @struct RadarInfo
 * @brief Target data struct which is detected from radar sensing node
 * @details This structure is a radar buffer which stores only data required by matching node among
 * recived messege from radar sensing node
 * @author jimin cho
 * @date 2022.05.19
 * @version 0.1
 */
struct RadarInfo {
  /** @var target_id Unique ID number of detected target */
  int target_id;
  /** @var track_type track_type represent target is tracking or not tracking */
  int track_type;
  /** @var track_type
   * track_type means motion state of targer. motion state is consist of
   * " Obstacle(1), Overhead target(2), Ground target(3), Departing target(4), Coming target(5),
   * ncertain(6), Right moving(7), Left moving(8), Mirror target(15) "
   */
  int target_type;
  /** @var v_x the relative longitudianl velocity of target */
  float v_x;
  /** @var v_x the relative lateral velocity of target */
  float v_y;
  /** @var longitudinal_x the relative longitudianl distance of target */
  float longitudinal_x;
  /** @var lateral_y the relative lateral distance of target */
  float lateral_y;
  /** @var u projected x coordinate to image plan */
  float u;
  /** @var v projected y coordinate to image plan */
  float v;
  /** @var rcs Radar cross-section */
  float rcs;

  float vertical_z;
  float width;
  float a_x;
  float confidence;
  unsigned char motion_state;
};
/**
 * @struct RadarInfoBuffer
 * @brief struct included information recived from radar sensing node
 * @details This structure stores timestamp and detected target information
 * @author jimin cho
 * @date 2022.05.19
 * @version 0.1
 */
struct MatchInfo {
  // Variables about radar data
  /** @var fusion_idUnique ID of fusioned data */
  unsigned int fusion_id;
  /** @var radar_id Unique ID number of detected target, which is assigned from radar sensing node
   */
  int radar_id;
  /** @var u Projected x coordinate to image plan */
  int u;
  /** @var v Projected y coordinate to image plan */
  int v;
  /** @var d_x The relative longitudianl distance of target */
  float d_x;
  /** @var v_x The relative longitudianl velocity of target  */
  float v_x;
  /** @var v_y The relative lateral velocity of target */
  float v_y;

  // Variables about perception data
  /** @var cam id Unique ID number of detected odject, which is assigned from perception node */
  int cam_id;
  /** @var class_id Type of detected object: car(0), bus(1), truck(2), pedestroan(10),... */
  unsigned char class_id;
  /** @var bbox Bounding box infomation of detected object,
   * it is consist of left-top point/width/height of bounding box */
  erae_std_msgs::msg::BoundingBox2D bbox;
  /** @var d_y The relative lateral distance of target */
  float d_y;

  /** @var track_mode Track_mode represents that it is need to add/track/delete at track buffer */
  TrackMode track_mode;
  /** @var sensor_type sensor_type represents that information of
   * matched data is consist of only perception or radar data or both */
  SensorType sensor_type;

  /** @var match_flag Variable to indicate whether input data and estimation data are matched */
  bool match_flag;
  /** @var unmatch_time Variables for expressing consecutive times
   * which estimation data is not matching with any real data. */
  float unmatch_time;

  // For Change FusionMode state
  // std::unique_ptr<FusionAlgorithm> fusion;
  // std::
  //  FusionMode fusion_mode;
  //  float radar_unmatching;
  //  float camera_unmatching;

  // For CarMaker
  float d_z;
  float a_x;
  int lane_info;
  float ttc;
  float confidence;
  float yaw_rate;
  unsigned char motion_state;
  int status;
  int speed;
  float width;

  // @Debug
  float cam_longitudinal;
  float cam_lateral;
  float radar_longitudinal;
  float radar_lateral;
};

template <typename T>
struct VectorData  // radar target data structure
{
  int timestamp;
  /** @var data_cnt total number of detected target on the same timestamp */
  // int data_cnt;
  std::vector<T> data;
  int size() { return data.size(); }
};

namespace PeriodSensor
{
constexpr float RADAR_PERIOD = 0.05;
constexpr float CAMERA_PERIOD = 0.033;
}  // namespace PeriodSensor

/* using MatchInfoPtrBuffer = VectorData<std::shared_ptr<struct MatchInfo>>;
using PerceptInfoBuffer = VectorData<struct PerceptInfo>;
using RadarInfoBuffer = VectorData<struct RadarInfo>; */

}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif