#ifndef FUSION__MATCHER_FUSION_ID_MANAGER_HPP_
#define FUSION__MATCHER_FUSION_ID_MANAGER_HPP_

#include <bitset>

namespace ebase
{
namespace fusion
{
namespace matcher
{
class FusionIdManager
{
  static constexpr unsigned int MAX = 256;

public:
  FusionIdManager();

  unsigned int GetId();
  void ResetId(unsigned int id);

private:
  std::bitset<MAX> id_;
  unsigned int last_index_;
};

}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_FUSION_ID_MANAGER_HPP_
