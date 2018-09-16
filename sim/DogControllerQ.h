#pragma once

#include "sim/BaseControllerQ.h"
#include "sim/DogController.h"

class cDogControllerQ : public virtual cDogController, public virtual cBaseControllerQ {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    cDogControllerQ();
    virtual ~cDogControllerQ();

  protected:
};
