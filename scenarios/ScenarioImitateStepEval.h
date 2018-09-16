#pragma once

#include "scenarios/ScenarioExpImitateStep.h"
#include "scenarios/ScenarioImitateTargetEval.h"

class cScenarioImitateStepEval : virtual public cScenarioImitateTargetEval, virtual public cScenarioExpImitateStep {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    cScenarioImitateStepEval();
    virtual ~cScenarioImitateStepEval();

    virtual std::string GetName() const;

  protected:
    virtual bool HasFallen() const;
};
