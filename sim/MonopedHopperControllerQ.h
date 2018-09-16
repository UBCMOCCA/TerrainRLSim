#pragma once

#include "sim/BaseControllerQ.h"
#include "sim/MonopedHopperController.h"

class cMonopedHopperControllerQ : public virtual cMonopedHopperController, public virtual cBaseControllerQ {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    cMonopedHopperControllerQ();
    virtual ~cMonopedHopperControllerQ();

  protected:
};
