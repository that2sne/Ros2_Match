#ifndef FUSION__MATCHER_SIMPLE_FUSION_HPP_
#define FUSION__MATCHER_SIMPLE_FUSION_HPP_

#include "fusion_algorithm.hpp"
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <list>
#include <map>
using namespace std::chrono;
//#define RAW_DATA
namespace ebase
{
namespace fusion
{
namespace matcher
{
using time_data = std::pair<system_clock::time_point, bool>;
class SimpleFusion : public FusionAlgorithm
{
public:
  SimpleFusion(const std::weak_ptr<MatchInfo> & match_info);
  ~SimpleFusion();

  virtual void TryFusion(SensorType && type);

private:
  void WriteMatchingInfo(SensorType & type);
  bool IsAccumulationDone();
  void UpdateStatus();

  float GetMatchingRate(const std::list<time_data> & data, int max_matching);
  void EraseData(std::list<time_data> & data, const int keeping_time);
  /// @brief  Data accumulation time to calculate the matching rate
  int accumulation_time_;
  int max_radar_matching_;
  int max_cam_matching_;
  float accept_rate_;
  /// @brief Matching rate to delete the object. When the matching rate falls below delete_rate, it
  /// is deleted.
  float delete_rate_;

  std::chrono::system_clock::time_point base_time_;
  std::list<time_data> cam_;
  std::list<time_data> radar_;
  rclcpp::Logger logger_;

#ifdef RAW_DATA
  std::map<int, int> debug_total_;
  std::map<int, int> debug_match_;
#endif
};
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_SIMPLE_FUSION_HPP_
