#ifndef FUSION__MATCHER_COMMAND_HPP_
#define FUSION__MATCHER_COMMAND_HPP_

#include "fusion_data.hpp"
#include "matcher/algorithm/matching.hpp"
namespace ebase
{
namespace fusion
{
namespace matcher
{

class Command
{
public:
  virtual void exectue(FusionData & fd) = 0;
};

template <typename T, typename A>
class CommandImpl : public Command
{
public:
  CommandImpl(T && data) : data_(data) {}
  virtual void exectue(FusionData & fd) { impl.Process(fd, data_); }

private:
  T data_;
  A impl;
};

using PerceptMathcingCommandT = CommandImpl<PerceptDataT, PerceptMatching>;
using RadarMathcingCommandT = CommandImpl<RadarDataT, RadarMatching>;
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_COMMAND_HPP_
