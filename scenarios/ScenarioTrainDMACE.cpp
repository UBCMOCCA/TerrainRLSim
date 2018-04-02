#include "ScenarioTrainDMACE.h"
#include "ScenarioExpDMACE.h"
#include "util/FileUtil.h"
#include "sim/BaseControllerDMACE.h"
#include "learning/DMACETrainer.h"
#include "learning/AsyncDMACETrainer.h"
#include "learning/DMACELearner.h"

//#define ENABLE_FIXED_TRAINER_TEMP
const double gFixedTrainerTemp = 1;

cScenarioTrainDMACE::cScenarioTrainDMACE()
{
}

cScenarioTrainDMACE::~cScenarioTrainDMACE()
{
}

std::string cScenarioTrainDMACE::GetName() const
{
	return "Train DMACE";
}

void cScenarioTrainDMACE::BuildTrainer(std::shared_ptr<cTrainerInterface>& out_trainer)
{
	const double gate_scale = 1;

	int num_frags = 0;
	int frag_size = 0;
	GetFragParams(num_frags, frag_size);
	
	if (mEnableAsyncMode)
	{
		auto trainer = std::shared_ptr<cAsyncDMACETrainer>(new cAsyncDMACETrainer());
		trainer->SetNumActionFrags(num_frags);
		trainer->SetActionFragSize(frag_size);
		trainer->SetGateScale(gate_scale);

#if defined(ENABLE_FIXED_TRAINER_TEMP)
		trainer->SetTemp(gFixedTrainerTemp);
#else
		trainer->SetTemp(mInitExpParams.mTemp);
#endif
		out_trainer = trainer;
	}
	else
	{
		auto trainer = std::shared_ptr<cDMACETrainer>(new cDMACETrainer());
		trainer->SetNumActionFrags(num_frags);
		trainer->SetActionFragSize(frag_size);
		trainer->SetGateScale(gate_scale);

#if defined(ENABLE_FIXED_TRAINER_TEMP)
		trainer->SetTemp(gFixedTrainerTemp);
#else
		trainer->SetTemp(mInitExpParams.mTemp);
#endif
		out_trainer = trainer;
	}
}

void cScenarioTrainDMACE::BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const
{
	out_exp = std::shared_ptr<cScenarioExp>(new cScenarioExpDMACE());
}

void cScenarioTrainDMACE::SetupTrainerCriticOffsetScale(const std::shared_ptr<cTrainerInterface>& out_trainer)
{
	cScenarioTrainCacla::SetupTrainerCriticOffsetScale(out_trainer);

	bool valid_init_model = out_trainer->HasCriticInitModel();
	if (!valid_init_model)
	{
		int critic_input_size = out_trainer->GetCriticInputSize();
		int critic_output_size = out_trainer->GetCriticOutputSize();

		const std::shared_ptr<cCharController>& ctrl = GetRefController();
		std::shared_ptr<cBaseControllerDMACE> dmace_ctrl = std::dynamic_pointer_cast<cBaseControllerDMACE>(ctrl);

		if (dmace_ctrl != nullptr)
		{
			Eigen::VectorXd critic_input_offset;
			Eigen::VectorXd critic_input_scale;
			dmace_ctrl->BuildCriticInputOffsetScale(critic_input_offset, critic_input_scale);
			
			Eigen::VectorXd critic_output_offset;
			Eigen::VectorXd critic_output_scale;
			dmace_ctrl->BuildCriticOutputOffsetScale(critic_output_offset, critic_output_scale);

			assert(critic_input_offset.size() == critic_input_size);
			assert(critic_input_scale.size() == critic_input_size);
			assert(critic_output_offset.size() == critic_output_size);
			assert(critic_output_scale.size() == critic_output_size);
			out_trainer->SetCriticInputOffsetScale(critic_input_offset, critic_input_scale);
			out_trainer->SetCriticOutputOffsetScale(critic_output_offset, critic_output_scale);
		}
		else
		{
			assert(false); // controller does not implement dmace interface
		}
	}
}

void cScenarioTrainDMACE::SetupTrainerActorOffsetScale(const std::shared_ptr<cTrainerInterface>& out_trainer)
{
	cScenarioTrain::SetupTrainerOffsetScale(out_trainer);
}

void cScenarioTrainDMACE::SetupLearner(const std::shared_ptr<cSimCharacter>& character, std::shared_ptr<cNeuralNetLearner>& out_learner) const
{
	cScenarioTrain::SetupLearner(character, out_learner);

	auto dmace_ctrl = std::dynamic_pointer_cast<cBaseControllerDMACE>(character->GetController());
	if (dmace_ctrl != nullptr)
	{
		auto ac_learner = std::static_pointer_cast<cACLearner>(out_learner);
		auto& critic = dmace_ctrl->GetCritic();
		ac_learner->SetCriticNet(critic.get());
	}
	else
	{
		assert(false); // controller does not support dmace
	}

	auto dmace_learner = std::static_pointer_cast<cDMACELearner>(out_learner);

#if defined(ENABLE_FIXED_TRAINER_TEMP)
	dmace_learner->SetTemp(gFixedTrainerTemp);
#else
	dmace_learner->SetTemp(mInitExpParams.mTemp);
#endif
}

void cScenarioTrainDMACE::GetFragParams(int& out_num_frags, int& out_frag_size) const
{
	auto ctrl = GetRefController();
	std::shared_ptr<cBaseControllerMACE> mace_ctrl = std::dynamic_pointer_cast<cBaseControllerMACE>(ctrl);

	if (mace_ctrl != nullptr)
	{
		out_num_frags = mace_ctrl->GetNumActionFrags();
		out_frag_size = mace_ctrl->GetActionFragSize();
	}
	else
	{
		assert(false); // controller does not implement MACE
	}
}

void cScenarioTrainDMACE::UpdateTrainer(const std::vector<tExpTuple>& tuples, int exp_id)
{
	auto learner = std::static_pointer_cast<cDMACELearner>(mLearners[exp_id]);
	int iter = learner->GetIter();
	cCharController::tExpParams exp_params = BuildExpParams(iter);

#if !defined(ENABLE_FIXED_TRAINER_TEMP)
	learner->SetTemp(exp_params.mTemp);
#endif

	cScenarioTrainCacla::UpdateTrainer(tuples, exp_id);
}
