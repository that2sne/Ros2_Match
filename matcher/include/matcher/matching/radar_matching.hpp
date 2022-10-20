#ifndef FUSION__MATCHER_MATCH_RADAR_INFO_HPP_
#define FUSION__MATCHER_MATCH_RADAR_INFO_HPP_

#include "matching.hpp"

namespace ebase
{
namespace fusion
{
namespace matcher
{
class RadarMatching : public Matching
{
public:
  virtual ~RadarMatching();
  RadarMatching(RadarInfoBuffer & radar);
  virtual void InitializeData(MatchInfoPtrBuffer & match_info);
  virtual SensorType GetType() { return type_; }
  virtual int GetDataCnt() { return radar_buff_.size(); }

protected:
  virtual void AddData(MatchInfoPtrBuffer & match_info, int real_index);
  virtual bool Match(
    MatchInfoPtrBuffer & match_buff_, int real_index, int match_index, float & min_distance);
  virtual bool Update(MatchInfoPtrBuffer & match_buff_, int real_index, int match_index);

private:
  RadarInfoBuffer radar_buff_;
  SensorType type_ = SensorType::kRadarData;
};
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_MATCH_RADAR_INFO_HPP_