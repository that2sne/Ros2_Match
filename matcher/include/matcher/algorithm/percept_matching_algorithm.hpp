#ifndef FUSION__MATCHER_PERCEPT_MATCHING_ALGORITHM_HPP_
#define FUSION__MATCHER_PERCEPT_MATCHING_ALGORITHM_HPP_
#include "matcher/fusion_data.hpp"
#include "matcher/percept_data.hpp"

namespace ebase
{
namespace fusion
{
namespace matcher
{
class PerceptMatchingAlgorithm
{
public:
  void Process(FusionData & fd, const PerceptData & rd);

private:
  void AddData(FusionData & fd, const PerceptInfo & ri);
  bool Match(MatchInfo & mi, const PerceptInfo & ri, float & min_distance);
  bool Update(MatchInfo & mi, const PerceptInfo & ri);
  void MatchOrUpdate(FusionData & fd, const PerceptData & rd);
  void InitializeData(FusionData & fd, const PerceptData & rd);
  SensorType GetType() { return SensorType::kCameraData; }
};
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_PERCEPT_MATCHING_ALGORITHM_HPP_