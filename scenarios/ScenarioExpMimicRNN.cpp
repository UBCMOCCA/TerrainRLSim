#include "ScenarioExpMimicRNN.h"
#include "learning/RNNTrainer.h"
#include "sim/CtRNNController.h"

cScenarioExpMimicRNN::cScenarioExpMimicRNN() {}

cScenarioExpMimicRNN::~cScenarioExpMimicRNN() {}

std::string cScenarioExpMimicRNN::GetName() const { return "Mimic RNN Exploration"; }

int cScenarioExpMimicRNN::GetNumWarmupCycles() const { return 0; }
