#include "matcher/parameter/match_parameter.hpp"

using namespace ebase::fusion::matcher;

MatchParameter::MatchParameter() {}

std::vector<ParameterDeclaration<int64_t>> MatchParameter::GetIntDeclaration()
{
  std::vector<ParameterDeclaration<int64_t>> vec;

  vec.push_back(ParameterDeclaration<int64_t>{"max_searching_range", 20});
  vec.push_back(ParameterDeclaration<int64_t>{"longit_gap", 5});
  vec.push_back(ParameterDeclaration<int64_t>{"lateral_gap",2});
  vec.push_back(ParameterDeclaration<int64_t>{"max_missing_distance", 5});
  
  return vec;
}
std::vector<ParameterDeclaration<double>> MatchParameter::GetDoubleDeclaration()
{
  std::vector<ParameterDeclaration<double>> vec;
  vec.push_back(ParameterDeclaration<double>{"base_num", 4.0});
  vec.push_back(ParameterDeclaration<double>{"matching_rate", 70.0});
  vec.push_back(ParameterDeclaration<double>{"unmatching_rate", 96.0});
  vec.push_back(ParameterDeclaration<double>{"duration", 1.0});

  return vec;
}
