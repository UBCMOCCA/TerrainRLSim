#pragma once
#include "sim/CtPDController.h"
#include "sim/CtTrackController.h"

class cCtPDTrackController : public virtual cCtTrackController, public virtual cCtPDController {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    cCtPDTrackController();
    virtual ~cCtPDTrackController();

    virtual void Init(cSimCharacter *character, const tVector &gravity, const std::string &param_file);

  protected:
};
