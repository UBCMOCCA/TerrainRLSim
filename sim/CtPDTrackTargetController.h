#pragma once
#include "sim/CtPDTrackController.h"
#include "sim/CtTargetController.h"

class cCtPDTrackTargetController : public virtual cCtTargetController, public virtual cCtPDTrackController {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    cCtPDTrackTargetController();
    virtual ~cCtPDTrackTargetController();

    virtual void Init(cSimCharacter *character);
    virtual void Init(cSimCharacter *character, const tVector &gravity, const std::string &param_file);

  protected:
    virtual int GetPoliStateSize() const;
    virtual int GetTargetPosStateOffset() const;
    virtual void BuildPoliState(Eigen::VectorXd &out_state) const;
};
