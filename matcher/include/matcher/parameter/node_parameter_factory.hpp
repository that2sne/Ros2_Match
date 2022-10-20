#ifndef FUSION__MATCHER_NODE_PARAMETER_FACTORY_HPP_
#define FUSION__MATCHER_NODE_PARAMETER_FACTORY_HPP_

#include <memory>

#include "node_parameter.hpp"

namespace ebase
{
namespace fusion
{
namespace matcher
{
template <class T>
class NodeParameterFactory
{
public:
  static std::shared_ptr<NodeParameter> get();
};

template <class T>
std::shared_ptr<NodeParameter> NodeParameterFactory<T>::get()
{
  static std::shared_ptr<NodeParameter> obj;
  if (obj == nullptr) obj = std::make_shared<T>();
  return obj;
}

}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_NODE_PARAMETER_FACTORY_HPP_