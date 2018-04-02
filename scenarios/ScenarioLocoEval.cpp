#include "scenarios/ScenarioLocoEval.h"
#include "scenarios/ScenarioExpLoco.h"

cScenarioLocoEval::cScenarioLocoEval() :
					cScenarioPoliEval(),
					cScenarioExpLoco()
{
}

cScenarioLocoEval::~cScenarioLocoEval()
{
}

std::string cScenarioLocoEval::GetName() const
{
	return "Locomotion Evaluation";
}