#ifndef FUSION__MATCHER_MATCH_PERCEPT_INFO_HPP_
#define FUSION__MATCHER_MATCH_PERCEPT_INFO_HPP_

#include "matching.hpp"

namespace ebase
{
namespace fusion
{
namespace matcher
{
class PerceptMatching : public Matching
{
public:
  virtual ~PerceptMatching();
  PerceptMatching(PerceptInfoBuffer & perceptoin_info);
  virtual void InitializeData(MatchInfoPtrBuffer & match_info);
  virtual SensorType GetType() { return type_; }
  virtual int GetDataCnt() { return perceptoin_info_buff_.size(); }

protected:
  virtual void AddData(MatchInfoPtrBuffer & match_info, int real_index);
  virtual bool Match(
    MatchInfoPtrBuffer & match_buff_, int real_index, int match_index, float & min_distance);
  virtual bool Update(MatchInfoPtrBuffer & match_buff_, int real_index, int match_index);

private:
  SensorType type_ = SensorType::kCameraData;
  PerceptInfoBuffer perceptoin_info_buff_;
};
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_MATCH_PERCEPT_INFO_HPP_