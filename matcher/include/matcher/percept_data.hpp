#ifndef FUSION__MATCHER_PECEPT_DATA_HPP_
#define FUSION__MATCHER_PECEPT_DATA_HPP_

#include "common.hpp"
#include "erae_perception_msgs/msg/fcw.hpp"

#include <vector>
using std::vector;

class PerceptData
{
public:
  PerceptData(const erae_perception_msgs::msg::FCW & fcw);

private:
  vector<struct PerceptInfo> data_;
};

#endif  // FUSION__MATCHER_PECEPT_DATA_HPP_