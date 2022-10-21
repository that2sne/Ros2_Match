#ifndef FUSION__MATCHER_RADAR_MATCHING_COMMAND_HPP_
#define FUSION__MATCHER_RADAR_MATCHING_COMMAND_HPP_

#include "matching_command.hpp"
#include "radar_data.hpp"

namespace ebase
{
namespace fusion
{
namespace matcher
{
class RadarMathcingCommand : public MatchingCommand
{
public:
  RadarMathcingCommand(RadarData && data);
  virtual void exectue(FusionData & fd) override;

private:
  RadarData data_;
};
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_RADAR_MATCHING_COMMAND_HPP_