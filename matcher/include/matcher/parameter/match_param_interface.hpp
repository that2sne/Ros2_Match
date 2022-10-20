#ifndef FUSION__MATCHER_MATCH_PARAM_INTERFACE_HPP_
#define FUSION__MATCHER_MATCH_PARAM_INTERFACE_HPP_
#include "match_parameter.hpp"
#include "node_parameter_factory.hpp"

// parameter access interface list
namespace ebase
{
namespace fusion
{
namespace matcher
{
using MatachParamShardPtr = NodeParameterFactory<MatchParameter>;
// NodeParameterFactory<MatchParameter>::get()->
/// maximum range of expansion pixel for definition searching region
inline int MatchParamMaxSearchingRange()
{
  return MatachParamShardPtr::get()->GetInt("max_searching_range");
}

/// threshold about difference distance between longitudinal distances of estimation and input data
inline int MatchParamLongitGap() { return MatachParamShardPtr::get()->GetInt("longit_gap"); }

/// threshold about difference distance between lateral distances of estimation and input data
inline int MatchParamLateralGap() { return MatachParamShardPtr::get()->GetInt("lateral_gap"); }

/// threshold about unmatching time for delete function
inline int MatchParamMaxMissingDistance()
{
  return MatachParamShardPtr::get()->GetInt("max_missing_distance");
}

/// base number of exponential function for definition searching region
inline float MatchParamBaseNum() { return MatachParamShardPtr::get()->GetDouble("base_num"); }

/// accept matching rate to determine status of fusion_mode changes
inline float MatchParamMatchingRate() { 
  return MatachParamShardPtr::get()->GetDouble("matching_rate"); 
}

/// accept unmatching rate to determin status of fusion_mode changes
inline float MatchParamUnMatchingRate() { 
  return MatachParamShardPtr::get()->GetDouble("unmatching_rate"); 
}

/// duration time for checking status of fusion_mode changes
inline float MatchParamDuration() { 
  return MatachParamShardPtr::get()->GetDouble("duration"); 
}

}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_MATCH_PARAM_INTERFACE_HPP_