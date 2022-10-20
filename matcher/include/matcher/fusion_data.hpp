#ifndef FUSION__MATCHER_FUSION_DATA_HPP_
#define FUSION__MATCHER_FUSION_DATA_HPP_

#include "common.hpp"

#include <list>
using std::list;

class FusionData
{
private:
  list<struct MatchInfo> data_;
};

#endif  // FUSION__MATCHER_FUSION_DATA_HPP_