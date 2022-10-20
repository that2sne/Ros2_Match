#ifndef FUSION__MATCHER_RELATIVE_SPEED_FUSION_HPP_
#define FUSION__MATCHER_RELATIVE_SPEED_FUSION_HPP_

#include "fusion_algorithm.hpp"

namespace ebase
{
namespace fusion
{
namespace matcher
{
class RelativeSpeedFusion : public FusionAlgorithm
{
public:
  RelativeSpeedFusion(const std::weak_ptr<MatchInfo> & match_info);
  ~RelativeSpeedFusion();

  virtual void TryFusion(SensorType && type);
  
private:
  
  FusionMode ChangeFromReadyState(MatchInfo & match_data, SensorType kind_of_input_data);
  FusionMode ChangeFromUnFusionState(MatchInfo & match_data, SensorType kind_of_input_data);
  FusionMode ChangeFromFusionState(MatchInfo & match_data, SensorType kind_of_input_data);

  float radar_unmatching_;
  float camera_unmatching_;
  FusionMode current_status_;
  FusionMode next_status_;
};
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_RELATIVE_SPEED_FUSION_HPP_
