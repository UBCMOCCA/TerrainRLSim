#include "DrawScenarioMimicRNN.h"
#include "scenarios/ScenarioMimicRNN.h"

cDrawScenarioMimicRNN::cDrawScenarioMimicRNN(cCamera& cam)
	: cDrawScenarioMimic(cam)
{
}

cDrawScenarioMimicRNN::~cDrawScenarioMimicRNN()
{
}

void cDrawScenarioMimicRNN::BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioMimicRNN>(new cScenarioMimicRNN());
}