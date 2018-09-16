#pragma once

#include "scenarios/ScenarioExpLoco.h"
#include "scenarios/ScenarioPoliEval.h"

class cScenarioLocoEval : virtual public cScenarioPoliEval, virtual public cScenarioExpLoco {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    cScenarioLocoEval();
    virtual ~cScenarioLocoEval();

    virtual std::string GetName() const;

  protected:
};
