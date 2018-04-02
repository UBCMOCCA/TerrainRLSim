#include "ScenarioMimicRNN.h"
#include "ScenarioExpMimicRNN.h"
#include "learning/RNNTrainer.h"
#include "learning/AsyncRNNTrainer.h"

cScenarioMimicRNN::cScenarioMimicRNN()
{
}

cScenarioMimicRNN::~cScenarioMimicRNN()
{
}

std::string cScenarioMimicRNN::GetName() const
{
	return "Mimic RNN";
}

void cScenarioMimicRNN::BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const
{
	out_exp = std::shared_ptr<cScenarioExpMimicRNN>(new cScenarioExpMimicRNN());
}

void cScenarioMimicRNN::BuildTrainer(std::shared_ptr<cTrainerInterface>& out_trainer)
{
	if (mEnableAsyncMode)
	{
		auto trainer = std::shared_ptr<cAsyncRNNTrainer>(new cAsyncRNNTrainer());
		out_trainer = trainer;
	}
	else
	{
		auto trainer = std::shared_ptr<cRNNTrainer>(new cRNNTrainer());
		out_trainer = trainer;
	}
}