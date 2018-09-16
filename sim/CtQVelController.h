#pragma once

#include "sim/CtQController.h"
#include "sim/CtVelController.h"

class cCtQVelController : public virtual cCtVelController, public virtual cCtQController {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    cCtQVelController();
    virtual ~cCtQVelController();

    virtual void Init(cSimCharacter *character, const tVector &gravity, const std::string &param_file);

  protected:
};
