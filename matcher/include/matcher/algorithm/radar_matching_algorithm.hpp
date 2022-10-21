#ifndef FUSION__MATCHER_RADAR_MATCHING_ALGORITHM_HPP_
#define FUSION__MATCHER_RADAR_MATCHING_ALGORITHM_HPP_
#include "matcher/fusion_data.hpp"
#include "matcher/radar_data.hpp"

namespace ebase
{
namespace fusion
{
namespace matcher
{
class RaderMatchingAlgorithm
{
public:
  void Process(FusionData & fd, const RadarData & rd);

private:
  void AddData(FusionData & fd, const RadarInfo & ri);
  bool Match(MatchInfo & mi, const RadarInfo & ri, float & min_distance);
  bool Update(MatchInfo & mi, const RadarInfo & ri);
  void MatchOrUpdate(FusionData & fd, const RadarData & rd);
  void InitializeData(FusionData & fd, const RadarData & rd);
  SensorType GetType() { return SensorType::kRadarData; }
};

}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_RADAR_MATCHING_ALGORITHM_HPP_