#ifndef FUSION__MATCHER_MATCH_OBJECT_HPP_
#define FUSION__MATCHER_MATCH_OBJECT_HPP_

#include "matcher/common.hpp"
#include "matcher/enum_class.hpp"
#include "matcher/fusion_id_manager.hpp"
namespace ebase
{
namespace fusion
{
namespace matcher
{
class Matching
{
public:
  virtual ~Matching();
  virtual void InitializeData(MatchInfoPtrBuffer & match_info) = 0;
  virtual void MatchOrUpdate(MatchInfoPtrBuffer & match_info);

  virtual SensorType GetType() = 0;
  virtual int GetDataCnt() = 0;

  void Attach(FusionIdManager * id);

protected:
  virtual void AddData(MatchInfoPtrBuffer & match_info, int real_index) = 0;
  virtual bool Match(
    MatchInfoPtrBuffer & match_buff_, int real_index, int match_index, float & min_distance) = 0;
  virtual bool Update(MatchInfoPtrBuffer & match_buff_, int real_index, int match_index) = 0;

  FusionIdManager * fusion_id_;

private:
};
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_MATCH_OBJECT_HPP_