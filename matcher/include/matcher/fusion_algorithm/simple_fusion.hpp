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
  struct Accumulation_data
  {
    std::chrono::system_clock::time_point base_time_;
    std::list<time_data> cam_;
    std::list<time_data> radar_;
    Accumulation_data() { base_time_ = std::chrono::system_clock::now(); }
  };
  SimpleFusion(const rclcpp::Clock::SharedPtr & clock, const rclcpp::Logger & logger);
  ~SimpleFusion();

  virtual void TryFusion(FusionData & dt, SensorType && type) override;
  virtual void Delete(int fusion_id) override;

private:
  void WriteMatchingInfo(Accumulation_data & dt, const MatchInfo & mi, const SensorType & type);
  bool IsAccumulationDone(Accumulation_data & dt);
  void UpdateStatus(Accumulation_data & dt, MatchInfo & mi);

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
  std::map<int, Accumulation_data> data_;

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

#ifdef RAW_DATA
  std::map<int, int> debug_total_;
  std::map<int, int> debug_match_;
#endif
};
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_SIMPLE_FUSION_HPP_
