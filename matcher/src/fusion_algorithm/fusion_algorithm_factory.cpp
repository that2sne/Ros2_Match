#include "matcher/fusion_algorithm/fusion_algorithm_factory.hpp"

using namespace ebase::fusion::matcher;

std::shared_ptr<FusionAlgorithmFactory> & FusionAlgorithmFactory::getInstance()
{
  static std::shared_ptr<FusionAlgorithmFactory> instance_;
  if (instance_ == nullptr) {
    instance_.reset(new FusionAlgorithmFactory());
  }
  return instance_;
}

std::unique_ptr<FusionAlgorithm> FusionAlgorithmFactory::CreateFusionAlgorithm()
{
  auto fac = FusionAlgorithmFactory::getInstance();
  switch (fac->type_) {
    /* case FusionAlgorithmType::kVehicleSpeedFusion:
      return std::make_unique<VehicleSpeedFusion>(match_info);
    case FusionAlgorithmType::kRelativeSpeedFusion:
      return std::make_unique<RelativeSpeedFusion>(match_info) */;
    case FusionAlgorithmType::kSimpleFusion:
      return std::make_unique<SimpleFusion>();
  }
  return nullptr;
}

void FusionAlgorithmFactory::SetAlgorithm(FusionAlgorithmType type)
{
  type_ = type;
}
