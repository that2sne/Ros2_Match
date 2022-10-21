#ifndef FUSION__MATCHER_MATCHING_COMMAND_HPP_
#define FUSION__MATCHER_MATCHING_COMMAND_HPP_

#include "command.hpp"
namespace ebase
{
namespace fusion
{
namespace matcher
{
class MatchingCommand : public Command
{
public:
  virtual void exectue(FusionData & fd) = 0;
};
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_MATCHING_COMMAND_HPP_