#ifndef FUSION__MATCHER_PERCEPT_MATCHING_COMMAND_HPP_
#define FUSION__MATCHER_PERCEPT_MATCHING_COMMAND_HPP_

#include "matching_command.hpp"
#include "percept_data.hpp"
namespace ebase
{
namespace fusion
{
namespace matcher
{
class PerceptMathcingCommand : public MatchingCommand
{
public:
  PerceptMathcingCommand(PerceptData && data);
  virtual void exectue(FusionData & fd) override;

private:
  PerceptData data_;
};
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_PERCEPT_MATCHING_COMMAND_HPP_