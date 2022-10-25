#ifndef FUSION__MATCHER_FUSION_ALGORITHM_FACTORY_HPP_
#define FUSION__MATCHER_FUSION_ALGORITHM_FACTORY_HPP_

#include "fusion_algorithm.hpp"
//#include "relative_speed_fusion.hpp"
#include "simple_fusion.hpp"
//#include "vehicle_speed_fusion.hpp"
namespace ebase
{
namespace fusion
{
namespace matcher
{
class FusionAlgorithmFactory
{
public:
  // enum class FusionAlgorithmType { kSimpleFusion };
  static std::shared_ptr<FusionAlgorithm> & GetInstance();
  static void Set(std::shared_ptr<FusionAlgorithm> && rhs);

private:
  static std::shared_ptr<FusionAlgorithm> algo_;
};
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_FUSION_ALGORITHM_FACTORY_HPP_