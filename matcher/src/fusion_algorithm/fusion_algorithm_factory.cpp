#include "matcher/fusion_algorithm/fusion_algorithm_factory.hpp"

using namespace ebase::fusion::matcher;
std::shared_ptr<FusionAlgorithm> FusionAlgorithmFactory::algo_;

std::shared_ptr<FusionAlgorithm> & FusionAlgorithmFactory::GetInstance()
{
  return algo_;
}

void FusionAlgorithmFactory::Set(std::shared_ptr<FusionAlgorithm> && rhs)
{
  algo_ = std::move(rhs);
}
