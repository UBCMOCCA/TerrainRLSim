#pragma once

#include "scenarios/ScenarioExpImitateTarget.h"
#include "scenarios/ScenarioImitateEval.h"

class cScenarioImitateTargetEval : virtual public cScenarioImitateEval, virtual public cScenarioExpImitateTarget {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    cScenarioImitateTargetEval();
    virtual ~cScenarioImitateTargetEval();

    virtual std::string GetName() const;

  protected:
};
