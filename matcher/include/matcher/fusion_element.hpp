#ifndef FUSION__MATCHER_FUSION_ELEMENT_HPP_
#define FUSION__MATCHER_FUSION_ELEMENT_HPP_

#include <functional>
#include <vector>

#include "common.hpp"
#include "erae_perception_msgs/msg/fcw.hpp"
#include "erae_sensor_msgs/msg/mrr_info_array.hpp"

namespace ebase
{
namespace fusion
{
namespace matcher
{
using std::vector;
template <typename T, typename V>
class FusionElement
{
private:
  vector<T> data_;

public:
  FusionElement(const V & rhs, std::function<vector<T>(const V &)> func);
  size_t GetSize() const { return data_.size(); }
  decltype(data_.cbegin()) begin() const { return data_.cbegin(); }
  decltype(data_.cend()) end() const { return data_.cend(); }

  decltype(data_.begin()) begin() { return data_.begin(); }
  decltype(data_.end()) end() { return data_.end(); }
};
template <typename T, typename V>
FusionElement<T, V>::FusionElement(const V & rhs, std::function<vector<T>(const V &)> func)
{
  data_ = std::move(func(rhs));
}
using PerceptDataT = FusionElement<struct PerceptInfo, erae_perception_msgs::msg::FCW>;
using RadarDataT = FusionElement<struct RadarInfo, erae_sensor_msgs::msg::MrrInfoArray>;

}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_FUSION_ELEMENT_HPP_