#ifndef FUSION__MATCHER_PECEPT_DATA_HPP_
#define FUSION__MATCHER_PECEPT_DATA_HPP_

#include <functional>
#include <vector>

#include "common.hpp"
#include "erae_perception_msgs/msg/fcw.hpp"

namespace ebase
{
namespace fusion
{
namespace matcher
{
using std::vector;

class PerceptData
{
private:
  vector<struct PerceptInfo> data_;

public:
  PerceptData(const erae_perception_msgs::msg::FCW & fcw);
  size_t GetSize() const { return data_.size(); }
  decltype(data_.cbegin()) begin() const { return data_.cbegin(); }
  decltype(data_.cend()) end() const { return data_.cend(); }

  decltype(data_.begin()) begin() { return data_.begin(); }
  decltype(data_.end()) end() { return data_.end(); }
};
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_PECEPT_DATA_HPP_