#include "DrawScenarioMimicEval.h"
#include "scenarios/ScenarioMimicEval.h"

cDrawScenarioMimicEval::cDrawScenarioMimicEval(cCamera& cam)
	: cDrawScenarioPoliEval(cam)
{
}

cDrawScenarioMimicEval::~cDrawScenarioMimicEval()
{
}

void cDrawScenarioMimicEval::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioMimicEval>(new cScenarioMimicEval());
}