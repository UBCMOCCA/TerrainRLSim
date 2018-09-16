#include "ScenarioImitateTargetEval.h"
#include "scenarios/ScenarioExpImitateTarget.h"
#include "sim/CtPhaseController.h"
#include "sim/CtTargetController.h"

cScenarioImitateTargetEval::cScenarioImitateTargetEval() : cScenarioExpImitateTarget(), cScenarioImitateEval() {}

cScenarioImitateTargetEval::~cScenarioImitateTargetEval() {}

std::string cScenarioImitateTargetEval::GetName() const { return "Imitate Target Evaluation"; }
