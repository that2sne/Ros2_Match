#ifndef FUSION__MATCHER_FUSION_ALGORITHM_HPP_
#define FUSION__MATCHER_FUSION_ALGORITHM_HPP_

#include "matcher/common.hpp"
#include "matcher/enum_class.hpp"

#include <chrono>
namespace ebase
{
namespace fusion
{
namespace matcher
{
struct MatchInfo;
class FusionAlgorithm
{
public:
  FusionAlgorithm(const std::weak_ptr<MatchInfo> & match_info);
  ~FusionAlgorithm();

  virtual void TryFusion(SensorType && type) = 0;

protected:
  std::weak_ptr<MatchInfo> match_info_;
};
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_FUSION_ALGORITHM_HPP_
