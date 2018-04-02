#include "DrawScenarioTrainMACEDPG.h"
#include "scenarios/ScenarioTrainMACEDPG.h"

cDrawScenarioTrainMACEDPG::cDrawScenarioTrainMACEDPG(cCamera& cam)
	: cDrawScenarioTrainDPG(cam)
{
}

cDrawScenarioTrainMACEDPG::~cDrawScenarioTrainMACEDPG()
{
}

void cDrawScenarioTrainMACEDPG::BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioTrainDPG>(new cScenarioTrainMACEDPG());
}