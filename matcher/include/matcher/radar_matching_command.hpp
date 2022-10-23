#ifndef FUSION__MATCHER_RADAR_MATCHING_COMMAND_HPP_
#define FUSION__MATCHER_RADAR_MATCHING_COMMAND_HPP_

#include "fusion_element.hpp"
#include "matching_command.hpp"

namespace ebase
{
namespace fusion
{
namespace matcher
{
class RadarMathcingCommand : public MatchingCommand
{
public:
  RadarMathcingCommand(RadarDataT && data);
  virtual void exectue(FusionData & fd) override;

private:
  RadarDataT data_;
};
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_RADAR_MATCHING_COMMAND_HPP_