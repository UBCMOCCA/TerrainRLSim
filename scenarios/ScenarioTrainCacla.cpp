#include "ScenarioTrainCacla.h"
#include "ScenarioExpCacla.h"
#include "util/FileUtil.h"
#include "sim/BaseControllerCacla.h"
#include "sim/CtStochController.h"
#include "learning/CaclaTrainer.h"
#include "learning/SARSATrainer.h"
#include "learning/StochPGTrainer.h"
#include "learning/AsyncCaclaTrainer.h"
#include "learning/AsyncSARSATrainer.h"
#include "learning/ACLearner.h"

cScenarioTrainCacla::cScenarioTrainCacla()
{
	mTrainerParams.mPoolSize = 1;
	mTrainerType = eCaclaTrainerTypeVal;
}

cScenarioTrainCacla::~cScenarioTrainCacla()
{
}

void cScenarioTrainCacla::ResetCriticWeights()
{
	{
		auto trainer = std::dynamic_pointer_cast<cACTrainer>(mTrainer);
		if (trainer != nullptr)
		{
			trainer->ResetCriticWeights();
		}
	}
	
	{
		auto trainer = std::dynamic_pointer_cast<cAsyncACTrainer>(mTrainer);
		if (trainer != nullptr)
		{
			trainer->ResetCriticWeights();
		}
	}
}

void cScenarioTrainCacla::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioTrain::ParseArgs(parser);

	std::string type_str = "";
	parser->ParseString("cacla_trainer_type", type_str);
	ParseTrainerType(type_str, mTrainerType);
}

std::string cScenarioTrainCacla::GetName() const
{
	return "Train Cacla";
}

void cScenarioTrainCacla::ParseTrainerType(const std::string& str, eCaclaTrainerType& out_mode) const
{
	if (str == "")
	{
	}
	else if (str == "val")
	{
		out_mode = eCaclaTrainerTypeVal;
	}
	else if (str == "sarsa")
	{
		out_mode = eCaclaTrainerTypeSARSA;
	}
	else if (str == "stoch")
	{
		out_mode = eCaclaTrainerTypeStoch;
	}
	else
	{
		printf("Unsupported trainer type: %s\n", str.c_str());
		assert(false);
	}
}

void cScenarioTrainCacla::InitTrainer()
{
	cScenarioTrain::InitTrainer();
	SetupTrainerActionCovar();
	SetupTrainerActionBounds();
}

void cScenarioTrainCacla::BuildTrainer(std::shared_ptr<cTrainerInterface>& out_trainer)
{
	if (mEnableAsyncMode)
	{
		switch (mTrainerType)
		{
		case (eCaclaTrainerTypeVal) :
			{
				auto trainer = std::shared_ptr<cAsyncCaclaTrainer>(new cAsyncCaclaTrainer());
				out_trainer = trainer;
				break;
			}
		case (eCaclaTrainerTypeSARSA) :
			{
				auto trainer = std::shared_ptr<cAsyncSARSATrainer>(new cAsyncSARSATrainer());
				out_trainer = trainer;
				break;
			}
		default:
			assert(false); // unsupported CACLA mode
			break;
		}
	}
	else
	{
		switch (mTrainerType)
		{
		case (eCaclaTrainerTypeVal) :
		{
			auto trainer = std::shared_ptr<cCaclaTrainer>(new cCaclaTrainer());
			out_trainer = trainer;
			break;
		}
		case (eCaclaTrainerTypeSARSA) :
		{
			auto trainer = std::shared_ptr<cSARSATrainer>(new cSARSATrainer());
			out_trainer = trainer;
			break;
		}
		case (eCaclaTrainerTypeStoch) :
		{
			auto trainer = std::shared_ptr<cStochPGTrainer>(new cStochPGTrainer());
			out_trainer = trainer;
			break;
		}
		default:
			assert(false); // unsupported CACLA mode
			break;
		}
	}
}

void cScenarioTrainCacla::SetupTrainer(const std::shared_ptr<cTrainerInterface>& out_trainer)
{
	cScenarioTrain::SetupTrainer(out_trainer);

	auto stoch_trainer = std::dynamic_pointer_cast<cStochPGTrainer>(out_trainer);
	if (stoch_trainer != nullptr)
	{
		SetupTrainerStoch(stoch_trainer);
	}
}

void cScenarioTrainCacla::SetupTrainerStoch(const std::shared_ptr<cStochPGTrainer>& out_trainer)
{
	cStochPGTrainer::tNoiseParams noise_params = out_trainer->GetNoiseParams();
	assert(mExpParams.mNoiseInternal == mInitExpParams.mNoiseInternal);

	auto stoch_ctrl = std::dynamic_pointer_cast<cCtStochController>(GetDefaultController());
	assert(stoch_ctrl != nullptr);

	Eigen::VectorXd output_offset;
	Eigen::VectorXd output_scale;
	out_trainer->GetActorOutputOffsetScale(output_offset, output_scale);

	int action_size = stoch_ctrl->GetPoliActionSize();
	int num_noise_units = stoch_ctrl->GetNumNoiseUnits();
	int noise_input_offset = stoch_ctrl->GetNoiseStateOffset();
	action_size -= num_noise_units;

	noise_params.mInputOffset = noise_input_offset;
	noise_params.mInputSize = num_noise_units;
	noise_params.mStdev = mInitExpParams.mNoiseInternal;
	noise_params.mKernel = output_scale.segment(0, action_size).asDiagonal();
	noise_params.mKernel *= noise_params.mKernel;

	out_trainer->SetNoiseParams(noise_params);
}

void cScenarioTrainCacla::BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const
{
	out_exp = std::shared_ptr<cScenarioExp>(new cScenarioExpCacla());
}

void cScenarioTrainCacla::SetupLearner(const std::shared_ptr<cSimCharacter>& character, std::shared_ptr<cNeuralNetLearner>& out_learner) const
{
	cScenarioTrain::SetupLearner(character, out_learner);

	auto ac_ctrl = std::dynamic_pointer_cast<cBaseControllerCacla>(character->GetController());
	if (ac_ctrl != nullptr)
	{
		auto ac_learner = std::static_pointer_cast<cACLearner>(out_learner);
		auto& critic = ac_ctrl->GetCritic();
		ac_learner->SetCriticNet(critic.get());
	}
	else
	{
		assert(false); // controller does not support cacla
	}
}

void cScenarioTrainCacla::SetupTrainerOffsetScale(const std::shared_ptr<cTrainerInterface>& out_trainer)
{
	SetupTrainerCriticOffsetScale(out_trainer);
	SetupTrainerActorOffsetScale(out_trainer);
}

void cScenarioTrainCacla::SetupTrainerCriticOffsetScale(const std::shared_ptr<cTrainerInterface>& out_trainer)
{
	bool valid_init_model = out_trainer->HasCriticInitModel();
	if (!valid_init_model)
	{
		int critic_input_size = out_trainer->GetCriticInputSize();
		int critic_output_size = out_trainer->GetCriticOutputSize();

		const std::shared_ptr<cCharController>& ctrl = GetRefController();
		std::shared_ptr<cBaseControllerCacla> cacla_ctrl = std::dynamic_pointer_cast<cBaseControllerCacla>(ctrl);

		if (cacla_ctrl != nullptr)
		{
			Eigen::VectorXd critic_input_offset;
			Eigen::VectorXd critic_input_scale;
			cacla_ctrl->BuildCriticInputOffsetScale(critic_input_offset, critic_input_scale);
			
			std::vector<cNeuralNet::eOffsetScaleType> scale_types;
			cacla_ctrl->BuildNNInputOffsetScaleTypes(scale_types);
			out_trainer->SetCriticInputOffsetScaleType(scale_types);

			Eigen::VectorXd critic_output_offset;
			Eigen::VectorXd critic_output_scale;
			cacla_ctrl->BuildCriticOutputOffsetScale(critic_output_offset, critic_output_scale);

			assert(critic_input_offset.size() == critic_input_size);
			assert(critic_input_scale.size() == critic_input_size);
			assert(critic_output_offset.size() == critic_output_size);
			assert(critic_output_scale.size() == critic_output_size);
			out_trainer->SetCriticInputOffsetScale(critic_input_offset, critic_input_scale);
			out_trainer->SetCriticOutputOffsetScale(critic_output_offset, critic_output_scale);
		}
		else
		{
			assert(false); // controller does not implement actor-critic interface
		}
	}
}

void cScenarioTrainCacla::SetupTrainerActorOffsetScale(const std::shared_ptr<cTrainerInterface>& out_trainer)
{
	bool valid_init_model = out_trainer->HasActorInitModel();
	if (!valid_init_model)
	{
		int actor_input_size = out_trainer->GetActorInputSize();
		int actor_output_size = out_trainer->GetActorOutputSize();

		const std::shared_ptr<cCharController>& ctrl = GetRefController();
		std::shared_ptr<cBaseControllerCacla> cacla_ctrl = std::dynamic_pointer_cast<cBaseControllerCacla>(ctrl);

		if (cacla_ctrl != nullptr)
		{
			Eigen::VectorXd actor_input_offset;
			Eigen::VectorXd actor_input_scale;
			cacla_ctrl->BuildActorInputOffsetScale(actor_input_offset, actor_input_scale);
			
			std::vector<cNeuralNet::eOffsetScaleType> scale_types;
			cacla_ctrl->BuildNNInputOffsetScaleTypes(scale_types);
			out_trainer->SetActorInputOffsetScaleType(scale_types);

			Eigen::VectorXd actor_output_offset;
			Eigen::VectorXd actor_output_scale;
			cacla_ctrl->BuildActorOutputOffsetScale(actor_output_offset, actor_output_scale);

			assert(actor_input_offset.size() == actor_input_size);
			assert(actor_input_scale.size() == actor_input_size);
			assert(actor_output_offset.size() == actor_output_size);
			assert(actor_output_scale.size() == actor_output_size);
			out_trainer->SetActorInputOffsetScale(actor_input_offset, actor_input_scale);
			out_trainer->SetActorOutputOffsetScale(actor_output_offset, actor_output_scale);
		}
		else
		{
			assert(false); // controller does not support CACLA
		}
	}
}

void cScenarioTrainCacla::BuildActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	const auto& ctrl = std::static_pointer_cast<cTerrainRLCharController>(GetDefaultController());
	ctrl->GetPoliActionBounds(out_min, out_max);
}

void cScenarioTrainCacla::SetupTrainerActionCovar()
{
	Eigen::VectorXd action_covar;
	BuildActionCovar(action_covar);

	auto cacla_trainer = std::dynamic_pointer_cast<cCaclaTrainer>(mTrainer);
	if (cacla_trainer != nullptr)
	{
		cacla_trainer->SetActionCovar(action_covar);
	}

	auto async_cacla_trainer = std::dynamic_pointer_cast<cAsyncCaclaTrainer>(mTrainer);
	if (async_cacla_trainer != nullptr)
	{
		async_cacla_trainer->SetActionCovar(action_covar);
	}
}

void cScenarioTrainCacla::SetupTrainerActionBounds()
{
	Eigen::VectorXd action_min;
	Eigen::VectorXd action_max;
	BuildActionBounds(action_min, action_max);

	auto cacla_trainer = std::dynamic_pointer_cast<cCaclaTrainer>(mTrainer);
	if (cacla_trainer != nullptr)
	{
		cacla_trainer->SetActionBounds(action_min, action_max);
	}

	auto async_cacla_trainer = std::dynamic_pointer_cast<cAsyncCaclaTrainer>(mTrainer);
	if (async_cacla_trainer != nullptr)
	{
		async_cacla_trainer->SetActionBounds(action_min, action_max);
	}
}

void cScenarioTrainCacla::BuildActionCovar(Eigen::VectorXd& out_covar) const
{
	const auto& ctrl = std::static_pointer_cast<cTerrainRLCharController>(GetDefaultController());
	ctrl->BuildActionExpCovar(out_covar);
}