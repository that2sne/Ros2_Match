#ifndef FUSION__MATCHER_ENUM_CLASS_HPP_
#define FUSION__MATCHER_ENUM_CLASS_HPP_
namespace ebase
{
namespace fusion
{
namespace matcher
{

enum class SensorType {
  // sensor_type
  kError = 0,
  kCameraData = 1,
  kRadarData = 2,
  kFusionData = 3
};

enum class FusionMode { kNone = 0, kReady = 1, kUnFusion = 2, kFusion = 3 };
/**
 * @brief Enum struct for representing track_mode
 * @details Enumerate the track_mode indicating how to process data input from the track node.
 * kInit: Add match data to track buffer, kUpdate: Update match data from prev track data,
 * kDelete: Delete match data to track buffer
 * @author jimin cho
 * @date 2022.05.24
 * @version 0.1
 */
enum class TrackMode { kInit = 1, kUpdate = 2, kDelete = 3 };
}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_ENUM_CLASS_HPP_