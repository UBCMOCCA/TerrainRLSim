#pragma once

#include "sim/BaseControllerQ.h"
#include "sim/RaptorController.h"

class cRaptorControllerQ : public virtual cRaptorController, public virtual cBaseControllerQ {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    cRaptorControllerQ();
    virtual ~cRaptorControllerQ();

  protected:
};
