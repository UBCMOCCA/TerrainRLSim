#pragma once
#include "sim/CtQPDController.h"
#include "sim/CtQTrackController.h"

class cCtQPDTrackController : public virtual cCtQTrackController, public virtual cCtQPDController {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    cCtQPDTrackController();
    virtual ~cCtQPDTrackController();

    virtual void Init(cSimCharacter *character, const tVector &gravity, const std::string &param_file);

  protected:
};
