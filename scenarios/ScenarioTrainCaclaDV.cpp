/*
 * ScenarioTrainCaclaDV.cpp
 *
 *  Created on: Oct 26, 2016
 *      Author: Glen
 */

#include "ScenarioTrainCaclaDV.h"
#include "learning/CaclaDVTrainer.h"
#include "learning/CaclaFDTrainer.h"
#include "learning/AsyncCaclaDVTrainer.h"
#include "sim/BaseControllerCaclaFD.h"
#include "learning/ACDLearner.h"
#include "util/FileUtil.h"
#include "learning/CaclaTrainer.h"
#include "learning/CaclaFDTrainer.h"

cScenarioTrainCaclaDV::cScenarioTrainCaclaDV() {
	// TODO Auto-generated constructor stub

}

cScenarioTrainCaclaDV::~cScenarioTrainCaclaDV() {
	// TODO Auto-generated destructor stub
}

void cScenarioTrainCaclaDV::SetupTrainerParams(cNeuralNetTrainer::tParams& out_params) const
{
	cScenarioTrainCacla::SetupTrainerParams(out_params);
	
	out_params.mForwardDynamicsNetFile = mNetFiles[cTerrainRLCtrlFactory::eNetFileForwardDynamics];
	out_params.mForwardDynamicsSolverFile = mNetFiles[cTerrainRLCtrlFactory::eNetFileForwardDynamicsSolver];
	out_params.mForwardDynamicsModelFile = mNetFiles[cTerrainRLCtrlFactory::eNetFileForwardDynamicsModel];
}

std::string cScenarioTrainCaclaDV::GetName() const
{
	return "Train CaclaDV";
}

void cScenarioTrainCaclaDV::SetupLearner(const std::shared_ptr<cSimCharacter>& character, std::shared_ptr<cNeuralNetLearner>& out_learner) const
{
	cScenarioTrainCacla::SetupLearner(character, out_learner);

	auto ac_ctrl = std::dynamic_pointer_cast<cBaseControllerCaclaFD>(character->GetController());
	if (ac_ctrl != nullptr)
	{
		auto ac_learner = std::static_pointer_cast<cACDLearner>(out_learner);
		auto& fd = ac_ctrl->GetForwardDynamics();
		ac_learner->SetForwardDynamicsNet(fd.get());
	}
	else
	{
		assert(false); // controller does not support cacla
	}
}

void cScenarioTrainCaclaDV::BuildTrainer(std::shared_ptr<cTrainerInterface>& out_trainer)
{
	if (mEnableAsyncMode)
	{
		auto trainer = std::shared_ptr<cAsyncCaclaDVTrainer>(new cAsyncCaclaDVTrainer());
		out_trainer = trainer;
	}
	else
	{
		auto trainer = std::shared_ptr<cCaclaFDTrainer>(new cCaclaFDTrainer());
		out_trainer = trainer;
	}
}

void cScenarioTrainCaclaDV::SetupTrainerOffsetScale(const std::shared_ptr<cTrainerInterface>& out_trainer)
{
	cScenarioTrainCacla::SetupTrainerOffsetScale(out_trainer);
	SetupTrainerForwardDynamicsOffsetScale(out_trainer);
}

void cScenarioTrainCaclaDV::SetupTrainerForwardDynamicsOffsetScale(const std::shared_ptr<cTrainerInterface>& out_trainer)
{
	bool valid_init_model = out_trainer->HasForwardDynamicsInitModel();
	if (!valid_init_model)
	{
		int fd_input_size = out_trainer->GetForwardDynamicsInputSize();
		int fd_output_size = out_trainer->GetForwardDynamicsOutputSize();

		const std::shared_ptr<cCharController>& ctrl = GetRefController();
		std::shared_ptr<cBaseControllerCacla> cacla_ctrl = std::dynamic_pointer_cast<cBaseControllerCacla>(ctrl);

		if (cacla_ctrl != nullptr)
		{
			// cCaclaFDTrainer * mTrainer_ = dynamic_cast<cCaclaFDTrainer *>(out_trainer);
			Eigen::VectorXd actor_input_offset;
			Eigen::VectorXd actor_input_scale;
			cacla_ctrl->BuildActorInputOffsetScale(actor_input_offset, actor_input_scale);

			Eigen::VectorXd actor_output_offset;
			Eigen::VectorXd actor_output_scale;
			cacla_ctrl->BuildActorOutputOffsetScale(actor_output_offset, actor_output_scale);

			Eigen::VectorXd fd_input_offset = Eigen::VectorXd::Ones(actor_input_offset.rows() + actor_output_offset.rows());
			Eigen::VectorXd fd_input_scale = Eigen::VectorXd::Ones(actor_input_offset.rows() + actor_output_offset.rows());
			// fd input State + Action
			fd_input_offset.segment(0, actor_input_offset.rows()) = actor_input_offset;
			fd_input_offset.segment(actor_input_offset.rows(), actor_output_offset.rows()) = actor_output_offset;
			fd_input_scale.segment(0, actor_input_scale.rows()) = actor_input_scale;
			fd_input_scale.segment(actor_input_scale.rows(), actor_output_scale.rows()) = actor_output_scale;

			// fd output is State
			Eigen::VectorXd fd_output_offset = actor_input_offset;
			Eigen::VectorXd fd_output_scale = actor_input_scale;

			assert((fd_input_offset.size() == fd_input_size) && "FD input scales to not match");
			assert((fd_input_scale.size() == fd_input_size) && "FD input scales to not match");
			assert((fd_output_offset.size() == fd_output_size) && "FD input scales to not match");
			assert((fd_output_scale.size() == fd_output_size) && "FD input scales to not match");
			out_trainer->SetForwardDynamicsInputOffsetScale(fd_input_offset, fd_input_scale);
			out_trainer->SetForwardDynamicsOutputOffsetScale(fd_output_offset, fd_output_scale);
		}
		else
		{
			assert(false); // controller does not support FD
		}
	}
}
