#ifndef FUSION__MATCHER_PERCEPT_MATCHING_COMMAND_HPP_
#define FUSION__MATCHER_PERCEPT_MATCHING_COMMAND_HPP_

#include "fusion_element.hpp"
#include "matching_command.hpp"
namespace ebase
{
namespace fusion
{
namespace matcher
{
class PerceptMathcingCommand : public MatchingCommand
{
public:
  PerceptMathcingCommand(PerceptDataT && data);
  virtual void exectue(FusionData & fd) override;

private:
  PerceptDataT data_;
};
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_PERCEPT_MATCHING_COMMAND_HPP_