#include "ScenarioTrainDPG.h"
#include "ScenarioExpDPG.h"
#include "util/FileUtil.h"
#include "sim/BaseControllerDPG.h"
#include "learning/DPGTrainer.h"
#include "learning/AsyncDPGTrainer.h"

cScenarioTrainDPG::cScenarioTrainDPG()
{
	mQDiff = 1;
	mDPGReg = 0.01;
}

cScenarioTrainDPG::~cScenarioTrainDPG()
{
}

void cScenarioTrainDPG::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioTrainCacla::ParseArgs(parser);
	parser->ParseDouble("trainer_dpg_q_diff", mQDiff);
	parser->ParseDouble("trainer_dpg_reg", mDPGReg);
	parser->ParseInt("trainer_net_pool_size", mTrainerParams.mPoolSize);
}

std::string cScenarioTrainDPG::GetName() const
{
	return "Train DPG";
}

void cScenarioTrainDPG::BuildTrainer(std::shared_ptr<cTrainerInterface>& out_trainer)
{
	if (mEnableAsyncMode)
	{
		auto trainer = std::shared_ptr<cAsyncDPGTrainer>(new cAsyncDPGTrainer());
		trainer->SetQDiff(mQDiff);
		trainer->SetDPGReg(mDPGReg);

		out_trainer = trainer;
	}
	else
	{
		auto trainer = std::shared_ptr<cDPGTrainer>(new cDPGTrainer());
		trainer->SetQDiff(mQDiff);
		trainer->SetDPGReg(mDPGReg);

		out_trainer = trainer;
	}
}

void cScenarioTrainDPG::BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const
{
	out_exp = std::shared_ptr<cScenarioExpDPG>(new cScenarioExpDPG());
}