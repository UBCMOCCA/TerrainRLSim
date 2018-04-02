#include "ScenarioTrainMACEDPG.h"
#include "ScenarioExpMACEDPG.h"
#include "util/FileUtil.h"
#include "sim/BaseControllerMACEDPG.h"
#include "learning/MACEDPGTrainer.h"
#include "learning/AsyncMACEDPGTrainer.h"

cScenarioTrainMACEDPG::cScenarioTrainMACEDPG()
{
	mTrainerTempScale = 1;
}

cScenarioTrainMACEDPG::~cScenarioTrainMACEDPG()
{
}

void cScenarioTrainMACEDPG::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioTrainDPG::ParseArgs(parser);
}

std::string cScenarioTrainMACEDPG::GetName() const
{
	return "Train MACE DPG";
}

void cScenarioTrainMACEDPG::BuildTrainer(std::shared_ptr<cTrainerInterface>& out_trainer)
{
	int num_frags = 0;
	int frag_size = 0;
	GetFragParams(num_frags, frag_size);
	
	if (mEnableAsyncMode)
	{
		auto trainer = std::shared_ptr<cAsyncMACEDPGTrainer>(new cAsyncMACEDPGTrainer());
		trainer->SetQDiff(mQDiff);
		trainer->SetDPGReg(mDPGReg);

		trainer->SetNumActionFrags(num_frags);
		trainer->SetActionFragSize(frag_size);
		trainer->SetTemp(mTrainerTempScale * mInitExpParams.mTemp);

		out_trainer = trainer;
	}
	else
	{
		auto trainer = std::shared_ptr<cMACEDPGTrainer>(new cMACEDPGTrainer());
		trainer->SetQDiff(mQDiff);
		trainer->SetDPGReg(mDPGReg);

		trainer->SetNumActionFrags(num_frags);
		trainer->SetActionFragSize(frag_size);
		trainer->SetTemp(mTrainerTempScale * mInitExpParams.mTemp);

		out_trainer = trainer;
	}
}

void cScenarioTrainMACEDPG::BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const
{
	out_exp = std::shared_ptr<cScenarioExpMACEDPG>(new cScenarioExpMACEDPG());
}

void cScenarioTrainMACEDPG::GetFragParams(int& out_num_frags, int& out_frag_size) const
{
	auto ctrl = GetRefController();
	std::shared_ptr<cBaseControllerMACEDPG> mace_ctrl = std::dynamic_pointer_cast<cBaseControllerMACEDPG>(ctrl);

	if (mace_ctrl != nullptr)
	{
		out_num_frags = mace_ctrl->GetNumActionFrags();
		out_frag_size = mace_ctrl->GetActionFragSize();
	}
	else
	{
		assert(false); // controller does not implement MACE DPG
	}
}

void cScenarioTrainMACEDPG::SetupLearner(const std::shared_ptr<cSimCharacter>& character, std::shared_ptr<cNeuralNetLearner>& out_learner) const
{
	cScenarioTrainDPG::SetupLearner(character, out_learner);
	auto mace_dpg_learner = std::static_pointer_cast<cMACEDPGLearner>(out_learner);
	mace_dpg_learner->SetTemp(mTrainerTempScale * mInitExpParams.mTemp);
}

void cScenarioTrainMACEDPG::UpdateTrainer(const std::vector<tExpTuple>& tuples, int exp_id)
{
	auto learner = std::static_pointer_cast<cMACEDPGLearner>(mLearners[exp_id]);
	int iter = learner->GetIter();
	cCharController::tExpParams exp_params = BuildExpParams(iter);
	learner->SetTemp(exp_params.mTemp * mTrainerTempScale);

	cScenarioTrainDPG::UpdateTrainer(tuples, exp_id);
}