#ifndef FUSION__MATCHER_ESTIMATION_HPP_
#define FUSION__MATCHER_ESTIMATION_HPP_

#include "command.hpp"
class Estimation : public Command
{
public:
  virtual void exectue() override;
};
#endif  // FUSION__MATCHER_ESTIMATION_HPP_