#ifndef FUSION__MATCHER_VEHICLE_SPEED_FUSION_HPP_
#define FUSION__MATCHER_VEHICLE_SPEED_FUSION_HPP_

#include "fusion_algorithm.hpp"

namespace ebase
{
namespace fusion
{
namespace matcher
{
class VehicleSpeedFusion : public FusionAlgorithm
{
public:
  VehicleSpeedFusion(const std::weak_ptr<MatchInfo> & match_info);
  ~VehicleSpeedFusion();

  virtual void TryFusion(SensorType && type);
  

private:
  FusionMode GetStatus();
  FusionMode GetNextStatus();
  bool IsChanging();
  void CheckProgress(SensorType & type);
  void TryChange(SensorType & type);
  bool IsTimeout(SensorType & type);
  // int CalculateHoldingTime(double ego_speed);
  int CalculateHoldingTime();
  FusionMode AssignNextStatus(MatchInfo & info, SensorType & type);
  FusionMode ChangeFromReadyState(MatchInfo & info, SensorType type);
  FusionMode ChangeFromFusionState(MatchInfo & info, SensorType type);
  FusionMode ChangeFromUnFusionState(MatchInfo & info, SensorType type);
  SensorType ChangeSensorType(MatchInfo & info);

  std::chrono::system_clock::time_point start_;
  // unsigned int holding_time_;
  unsigned int cam_holding_time_;
  unsigned int radar_holding_time_;
  unsigned int cam_count_;
  unsigned int radar_count_;
  unsigned int cam_total_count_;
  unsigned int radar_total_count_;

  SensorType next_type_;
  FusionMode current_status_;
  FusionMode next_status_;
};
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_VEHICLE_SPEED_FUSION_HPP_
