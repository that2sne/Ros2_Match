#ifndef FUSION__MATCHER_FUSION_ALGORITHM_HPP_
#define FUSION__MATCHER_FUSION_ALGORITHM_HPP_

#include "matcher/common.hpp"
#include "matcher/enum_class.hpp"
#include "matcher/fusion_data.hpp"

#include <chrono>
namespace ebase
{
namespace fusion
{
namespace matcher
{
struct MatchInfo;
class FusionData;
class FusionAlgorithm
{
public:
  FusionAlgorithm();
  ~FusionAlgorithm();

  virtual void TryFusion(FusionData & dt, SensorType && type) = 0;
  virtual void Delete(int fusion_id) = 0;

protected:
};
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_FUSION_ALGORITHM_HPP_
