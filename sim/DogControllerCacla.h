#pragma once

#include "sim/BaseControllerCacla.h"
#include "sim/DogController.h"

class cDogControllerCacla : public virtual cDogController, public virtual cBaseControllerCacla {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    cDogControllerCacla();
    virtual ~cDogControllerCacla();

    virtual void Init(cSimCharacter *character, const tVector &gravity, const std::string &param_file);

  protected:
    virtual void UpdateAction();
    virtual bool IsCurrActionCyclic() const;
};
