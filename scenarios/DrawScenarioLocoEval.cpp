#include "scenarios/DrawScenarioLocoEval.h"
#include "scenarios/ScenarioLocoEval.h"

cDrawScenarioLocoEval::cDrawScenarioLocoEval(cCamera& cam)
						: cDrawScenarioPoliEval(cam)
{
}

cDrawScenarioLocoEval::~cDrawScenarioLocoEval()
{
}

void cDrawScenarioLocoEval::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioLocoEval>(new cScenarioLocoEval());
}