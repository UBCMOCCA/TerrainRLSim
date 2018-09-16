#pragma once

#include "scenarios/ScenarioExpHike.h"
#include "scenarios/ScenarioImitateStepEval.h"

class cScenarioHikeEval : virtual public cScenarioImitateStepEval, virtual public cScenarioExpHike {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    cScenarioHikeEval();
    virtual ~cScenarioHikeEval();

    virtual std::string GetName() const;

  protected:
    virtual bool EnableLLCFeedbackReward() const;
};
