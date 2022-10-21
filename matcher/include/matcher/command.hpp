#ifndef FUSION__MATCHER_COMMAND_HPP_
#define FUSION__MATCHER_COMMAND_HPP_

#include "fusion_data.hpp"
namespace ebase
{
namespace fusion
{
namespace matcher
{
class Command
{
public:
  virtual void exectue(FusionData & fd) = 0;
};
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_COMMAND_HPP_