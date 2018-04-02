#include "OptScenarioLocoEval.h"
#include "scenarios/ScenarioLocoEval.h"

cOptScenarioLocoEval::cOptScenarioLocoEval()
{
}

cOptScenarioLocoEval::~cOptScenarioLocoEval()
{
}

void cOptScenarioLocoEval::BuildScene(int id, std::shared_ptr<cScenarioPoliEval>& out_scene) const
{
	auto eval_scene = std::shared_ptr<cScenarioLocoEval>(new cScenarioLocoEval());
	out_scene = eval_scene;
}