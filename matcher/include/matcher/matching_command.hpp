#ifndef FUSION__MATCHER_MATCHING_COMMAND_HPP_
#define FUSION__MATCHER_MATCHING_COMMAND_HPP_

#include "command.hpp"
class MatchingCommand : public Command
{
public:
  virtual void exectue() = 0;
};
#endif  // FUSION__MATCHER_MATCHING_COMMAND_HPP_