#pragma once

#include "scenarios/ScenarioExpSoccer.h"
#include "scenarios/ScenarioHikeEval.h"

class cScenarioSoccerEval : virtual public cScenarioHikeEval, virtual public cScenarioExpSoccer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    cScenarioSoccerEval();
    virtual ~cScenarioSoccerEval();

    virtual std::string GetName() const;

  protected:
};
