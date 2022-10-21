
#include "matcher/radar_matching_command.hpp"

#include "matcher/algorithm/radar_matching_algorithm.hpp"
using ebase::fusion::matcher::RadarMathcingCommand;
RadarMathcingCommand::RadarMathcingCommand(RadarData && data) : data_(data) {}

void RadarMathcingCommand::exectue(FusionData & fd)
{
  // 알고리즘생성하고 process 호출
  RaderMatchingAlgorithm a;
  a.Process(fd, data_);
  // 퓨전 상태 결정하는 알고리즘 호출
}