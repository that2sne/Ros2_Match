#ifndef FUSION__MATCHER_FUSION_ELEMENT_HPP_
#define FUSION__MATCHER_FUSION_ELEMENT_HPP_

#include "common.hpp"
#include "enum_class.hpp"
#include "erae_perception_msgs/msg/fcw.hpp"
#include "erae_sensor_msgs/msg/mrr_info_array.hpp"

#include <functional>
#include <vector>
namespace ebase
{
namespace fusion
{
namespace matcher
{
using std::vector;
template<typename T, typename V>
class FusionElement
{
public:
  vector<T> data_;

public:
  FusionElement(const V & rhs, std::function<vector<T>(const V &)> func);
  size_t GetSize() const { return data_.size(); }
  SensorType GetType() const
  {
    if (std::is_same<T, struct PerceptInfo>::value) {
      return SensorType::kCameraData;
    } else if (std::is_same<T, struct RadarInfo>::value) {
      return SensorType::kRadarData;
    }
    return SensorType::kError;
  }
  decltype(data_.cbegin()) begin() const { return data_.cbegin(); }
  decltype(data_.cend()) end() const { return data_.cend(); }

  decltype(data_.begin()) begin() { return data_.begin(); }
  decltype(data_.end()) end() { return data_.end(); }
};
template<typename T, typename V>
FusionElement<T, V>::FusionElement(const V & rhs, std::function<vector<T>(const V &)> func)
{
  data_ = std::move(func(rhs));
}
using PerceptData = FusionElement<struct PerceptInfo, erae_perception_msgs::msg::FCW>;
using RadarData = FusionElement<struct RadarInfo, erae_sensor_msgs::msg::MrrInfoArray>;

}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_FUSION_ELEMENT_HPP_