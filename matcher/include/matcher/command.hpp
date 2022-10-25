#ifndef FUSION__MATCHER_COMMAND_HPP_
#define FUSION__MATCHER_COMMAND_HPP_

#include "fusion_data.hpp"
#include "matcher/matching.hpp"
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

template<typename T, typename A>
class CommandImpl : public Command
{
public:
  CommandImpl(T && data, const A & cmd) : data_(data), impl(cmd) {}
  virtual void exectue(FusionData & fd) { impl->Process(fd, data_); }

private:
  T data_;
  A impl;
};

using PerceptMathcingCommand = CommandImpl<PerceptData, std::shared_ptr<PerceptMatching>>;
using RadarMathcingCommand = CommandImpl<RadarData, std::shared_ptr<RadarMatching>>;
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_COMMAND_HPP_
