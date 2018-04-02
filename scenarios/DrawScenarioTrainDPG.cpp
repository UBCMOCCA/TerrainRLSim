#include "DrawScenarioTrainDPG.h"
#include "scenarios/ScenarioTrainDPG.h"

cDrawScenarioTrainDPG::cDrawScenarioTrainDPG(cCamera& cam)
	: cDrawScenarioTrainCacla(cam)
{
}

cDrawScenarioTrainDPG::~cDrawScenarioTrainDPG()
{
}

void cDrawScenarioTrainDPG::BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioTrainDPG>(new cScenarioTrainDPG());
}