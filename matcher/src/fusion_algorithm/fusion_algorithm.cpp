#include "matcher/fusion_algorithm/fusion_algorithm.hpp"

using namespace ebase::fusion::matcher;

FusionAlgorithm::FusionAlgorithm(const std::weak_ptr<MatchInfo> & match_info)
: match_info_(match_info)
{
}

FusionAlgorithm::~FusionAlgorithm() {}
