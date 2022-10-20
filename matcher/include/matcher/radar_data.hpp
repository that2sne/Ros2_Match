#ifndef FUSION__MATCHER_RADAR_DATA_HPP_
#define FUSION__MATCHER_RADAR_DATA_HPP_

#include "common.hpp"
#include "erae_sensor_msgs/msg/mrr_info_array.hpp"

#include <vector>

namespace ebase
{
namespace fusion
{
namespace matcher
{
using std::vector;
class RadarData
{
public:
  RadarData(const erae_sensor_msgs::msg::MrrInfoArray & mrr);

private:
  vector<struct RadarInfo> data_;
};
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_RADAR_DATA_HPP_
