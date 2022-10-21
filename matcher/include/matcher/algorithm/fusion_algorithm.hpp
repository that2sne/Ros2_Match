#ifndef FUSION__MATCHER_FUSION_ALGORITHM_HPP_
#define FUSION__MATCHER_FUSION_ALGORITHM_HPP_
class FusionAlgorithm
{
public:
  virtual void TryFusion(SensorType && type) = 0;
};

#endif  // FUSION__MATCHER_FUSION_ALGORITHM_HPP_