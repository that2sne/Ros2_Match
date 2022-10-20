#ifndef FUSION__MATCHER_MATCH_PARAMETER_HPP_
#define FUSION__MATCHER_MATCH_PARAMETER_HPP_

#include "node_parameter.hpp"

namespace ebase
{
namespace fusion
{
namespace matcher
{

class MatchParameter : public NodeParameter
{
public:
  MatchParameter();

protected:
  virtual std::vector<ParameterDeclaration<int64_t>> GetIntDeclaration();
  virtual std::vector<ParameterDeclaration<double>> GetDoubleDeclaration();

private:
};
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_MATCH_PARAMETER_HPP_
