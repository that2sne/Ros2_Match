
#include "matcher/fusion_id_manager.hpp"

#include <stdexcept>

using namespace ebase::fusion::matcher;
FusionIdManager::FusionIdManager() : last_index_(-1) { id_.reset(0); }

unsigned int FusionIdManager::GetId()
{
  unsigned int temp = last_index_ + 1;
  bool find = false;
  for (; temp < FusionIdManager::MAX; temp++) {
    if (!id_.test(temp)) {
      find = true;
      break;
    }
  }
  if (!find) {
    for (temp = 0; temp < last_index_; temp++) {
      if (!id_.test(temp)) {
        find = true;
        break;
      }
    }
  }
  if (!find) {
    throw std::out_of_range("FusionAlgorithm id is out of range");
  }

  last_index_ = temp;
  id_.set(last_index_, true);
  return last_index_;
}

void FusionIdManager::ResetId(unsigned int id) { id_.set(id, false); }