#include "ScenarioExpMimicRNN.h"
#include "sim/CtRNNController.h"
#include "learning/RNNTrainer.h"

cScenarioExpMimicRNN::cScenarioExpMimicRNN()
{
}

cScenarioExpMimicRNN::~cScenarioExpMimicRNN()
{
}

std::string cScenarioExpMimicRNN::GetName() const
{
	return "Mimic RNN Exploration";
}

int cScenarioExpMimicRNN::GetNumWarmupCycles() const
{
	return 0;
}