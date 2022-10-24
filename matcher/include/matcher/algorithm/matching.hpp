#ifndef FUSION__MATCHER_MATCHING_HPP_
#define FUSION__MATCHER_MATCHING_HPP_
#include "matcher/fusion_data.hpp"
#include "matcher/fusion_element.hpp"

namespace ebase
{
namespace fusion
{
namespace matcher
{
template<typename T, typename I>
struct IMatch
{
  std::function<bool(MatchInfo & mi, const I & input_info)> Update;
  std::function<bool(MatchInfo & mi, const I & input_info, float & min_distance)> Match;
};

class Matching
{
public:
  template<typename T, typename I>
  static void Process(FusionData & fd, const T & input_data, IMatch<T, I> & func)
  {
    if (fd.GetSize() == 0 && input_data.GetSize() > 0) {
      InitializeData(fd, input_data);

    } else if (fd.GetSize() != 0 && input_data.GetSize() > 0) {
      MatchOrUpdate(fd, input_data, func);
    } else {
      // RCLCPP_ERROR(logger_, "There is no any input data...");
    }
    fd.UpdateStatus(input_data.GetType());
  }

private:
  template<typename T>
  static void InitializeData(FusionData & fd, const T & input_data);

  template<typename T, typename I>
  static void MatchOrUpdate(FusionData & fd, const T & input_data, IMatch<T, I> & func);

  template<typename T>
  static void AddData(FusionData & fd, const T & input_info);
};
class RadarMatching
{
public:
  static bool Update(MatchInfo & mi, const RadarInfo & input_info);
  static bool Match(MatchInfo & mi, const RadarInfo & input_info, float & min_distance);

  void Process(FusionData & fd, const RadarDataT & input_data)
  {
    IMatch<RadarDataT, RadarInfo> fn;
    fn.Update = &RadarMatching::Update;
    fn.Match = &RadarMatching::Match;

    Matching::Process(fd, input_data, fn);
  }
};

class PerceptMatching
{
public:
  static bool Update(MatchInfo & mi, const PerceptInfo & input_info);
  static bool Match(MatchInfo & mi, const PerceptInfo & input_info, float & min_distance);
  void Process(FusionData & fd, const PerceptDataT & input_data)
  {
    IMatch<PerceptDataT, PerceptInfo> fn;
    fn.Update = &PerceptMatching::Update;
    fn.Match = &PerceptMatching::Match;

    Matching::Process(fd, input_data, fn);
  }
};

template<typename T, typename I>
void Matching::MatchOrUpdate(FusionData & fd, const T & input_data, IMatch<T, I> & impl_)
{
  for (auto & i : fd) {
    i.match_flag = false;
  }

  for (auto & ri : input_data) {
    //  checking unmatch input data and any estimation
    float min_dist = 999999.999999;
    int prev_match_index = -1;
    bool match_flag = false;
    for (size_t match_index = 0; match_index < fd.GetSize();
         match_index++)  // estimation buffer loop
    {
      bool is_match = false;

      // Step 2: Checking that fusion flag of estimation data is same input data
      // not using estimation data
      if (
        fd.data_[match_index].match_flag == false ||
        (fd.data_[match_index].cam_id == -1 || fd.data_[match_index].radar_id == -1))
      {
        // When input data type is the same estimation sensor_type, update estimation data with
        // input data
        if (fd.data_[match_index].sensor_type == input_data.GetType()) {
          is_match = impl_.Update(fd.data_[match_index], ri);

          if (match_flag == true) is_match = true;
        }
        // When input data type is not same estimation sensor_type, matching estimation data with
        // input data
        else
        {
          is_match = impl_.Match(fd.data_[match_index], ri, min_dist);
          // fd.data_[match_index].sensor_type = SensorType::kFusionData;
          if (match_flag == true && is_match == true && prev_match_index != -1) {
            if (fd.data_[prev_match_index].sensor_type == input_data.GetType())
              fd.data_[prev_match_index].track_mode = TrackMode::kDelete;
          }
          if (match_flag == true) is_match = true;
        }

        if (is_match == true) {
          if (prev_match_index != -1 && prev_match_index != static_cast<int>(match_index)) {
            fd.data_[prev_match_index].match_flag = false;
            fd.data_[prev_match_index].unmatch_time = 10000;
            prev_match_index = match_index;
          }
          match_flag = true;

          if (min_dist == -1) break;
        }
      }
    }

    if (match_flag == false) {
      AddData(fd, ri);
    }
  }
}

template<typename T>
void Matching::InitializeData(FusionData & fd, const T & input_data)
{
  for (auto & i : input_data) {
    AddData(fd, i);  // same  fd += pd;
  }
/*   for (auto & mi : fd) {
    float match_box_area = (float)(mi.bbox.width * mi.bbox.height);
    //float percept_box_area = (float)(pi.width * pi.height);
    printf("match_box_area %f\n", match_box_area);
    fflush(stdout);
  }
 */
   
}
template<typename T>
void Matching::AddData(FusionData & fd, const T & input_info)
{
  fd += input_info;
}

}  // namespace matcher
}  // namespace fusion
}  // namespace ebase
#endif  // FUSION__MATCHER_MATCHING_HPP_