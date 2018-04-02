#include "DrawScenarioTrainLoco.h"
#include "scenarios/ScenarioTrainLoco.h"

cDrawScenarioTrainLoco::cDrawScenarioTrainLoco(cCamera& cam)
						: cDrawScenarioTrainCacla(cam)
{
}

cDrawScenarioTrainLoco::~cDrawScenarioTrainLoco()
{
}

void cDrawScenarioTrainLoco::BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioTrain>(new cScenarioTrainLoco());
}