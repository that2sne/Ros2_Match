
#include "matcher/percept_matching_command.hpp"

#include "matcher/algorithm/percept_matching_algorithm.hpp"
using ebase::fusion::matcher::PerceptMathcingCommand;
PerceptMathcingCommand::PerceptMathcingCommand(PerceptData && data) : data_(data) {}

void PerceptMathcingCommand::exectue(FusionData & fd)
{
  PerceptMatchingAlgorithm a;
  a.Process(fd, data_);
}