#pragma once
#include "sim/CtPDPhaseController.h"
#include "sim/CtTargetController.h"

class cCtPDPhaseTargetController : public virtual cCtTargetController, public virtual cCtPDPhaseController {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    cCtPDPhaseTargetController();
    virtual ~cCtPDPhaseTargetController();

    virtual void Init(cSimCharacter *character);
    virtual void Init(cSimCharacter *character, const tVector &gravity, const std::string &param_file);

  protected:
    virtual int GetPoliStateSize() const;
    virtual int GetTargetPosStateOffset() const;
    virtual void BuildPoliState(Eigen::VectorXd &out_state) const;
};
