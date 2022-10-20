#ifndef FUSION__MATCHER_PERCEPT_MATCHING_COMMAND_HPP_
#define FUSION__MATCHER_PERCEPT_MATCHING_COMMAND_HPP_

#include "matching_command.hpp"
class PerceptMathcingCommand : public MatchingCommand
{
public:
  virtual void exectue() override;
};
#endif  // FUSION__MATCHER_PERCEPT_MATCHING_COMMAND_HPP_