#ifndef FUSION__MATCHER_RADAR_DATA_HPP_
#define FUSION__MATCHER_RADAR_DATA_HPP_

#include <vector>

#include "common.hpp"
#include "erae_sensor_msgs/msg/mrr_info_array.hpp"

namespace ebase
{
namespace fusion
{
namespace matcher
{
using std::vector;
class RadarData
{
private:
  vector<struct RadarInfo> data_;

public:
  RadarData(const erae_sensor_msgs::msg::MrrInfoArray & mrr);
  size_t GetSize() const { return data_.size(); }
  decltype(data_.cbegin()) begin() const { return data_.cbegin(); }
  decltype(data_.cend()) end() const { return data_.cend(); }

  decltype(data_.begin()) begin() { return data_.begin(); }
  decltype(data_.end()) end() { return data_.end(); }
};
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_RADAR_DATA_HPP_
