#include "DrawScenarioTrainDMACE.h"
#include "scenarios/ScenarioTrainDMACE.h"

cDrawScenarioTrainDMACE::cDrawScenarioTrainDMACE(cCamera& cam)
	: cDrawScenarioTrainCacla(cam)
{
}

cDrawScenarioTrainDMACE::~cDrawScenarioTrainDMACE()
{
}

void cDrawScenarioTrainDMACE::BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioTrainDMACE>(new cScenarioTrainDMACE());
}