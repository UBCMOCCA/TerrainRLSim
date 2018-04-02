/*
 * BaseControllerCaclaFD.cpp
 *
 *  Created on: Nov 18, 2016
 *      Author: jocst
 */

#include "BaseControllerCaclaFD.h"

cBaseControllerCaclaFD::cBaseControllerCaclaFD() : cBaseControllerCacla() {
	// TODO Auto-generated constructor stub

}

cBaseControllerCaclaFD::~cBaseControllerCaclaFD() {
	// TODO Auto-generated destructor stub
}

int cBaseControllerCaclaFD::GetForwardDynamicsInputSize() const
{
	return GetPoliStateSize() + GetPoliActionSize();
}

int cBaseControllerCaclaFD::GetForwardDynamicsOutputSize() const
{
	return GetPoliStateSize();
}

void cBaseControllerCaclaFD::CopyForwardDynamicsNet(const cNeuralNet& net)
{
	mForwardDynamicsNet->CopyModel(net);
}

const std::unique_ptr<cNeuralNet>& cBaseControllerCaclaFD::GetForwardDynamics() const
{
	return mForwardDynamicsNet;
}

void cBaseControllerCaclaFD::Init(cSimCharacter* character)
{
	cBaseControllerCacla::Init(character);
	BuildForwardDynamicsNet(mForwardDynamicsNet);
}

void cBaseControllerCaclaFD::BuildForwardDynamicsNet(std::unique_ptr<cNeuralNet>& out_net) const
{
	out_net = std::unique_ptr<cNeuralNet>(new cNeuralNet());
}

bool cBaseControllerCaclaFD::LoadForwardDynamicsNet(const std::string& net_file)
{
	bool succ = true;
	// LoadNetIntern(net_file);
	mForwardDynamicsNet->Clear();
	mForwardDynamicsNet->LoadNet(net_file);

	int input_size = mForwardDynamicsNet->GetInputSize();
	int output_size = mForwardDynamicsNet->GetOutputSize();
	int state_size = GetNetInputSize() + GetNetOutputSize();
	int action_size = GetNetInputSize();

	if (output_size != action_size)
	{
		printf("Network output dimension does not match expected output size (%i vs %i).\n", output_size, action_size);
		succ = false;
	}

	if (input_size != state_size)
	{
		printf("Network input dimension does not match expected input size (%i vs %i).\n", input_size, state_size);
		succ = false;
	}

	if (succ)
	{
		Eigen::VectorXd input_offset;
		Eigen::VectorXd input_scale;
		BuildForwardDynamicsInputOffsetScale(input_offset, input_scale);
		SetForwardDynamicsInputOffsetScale(input_offset, input_scale);

		Eigen::VectorXd output_offset;
		Eigen::VectorXd output_scale;
		BuildForwardDynamicsOutputOffsetScale(output_offset, output_scale);
		SetForwardDynamicsOutputOffsetScale(output_offset, output_scale);
	}
	else
	{
		mNet->Clear();
		assert(false);
	}

	return succ;
}

void cBaseControllerCaclaFD::LoadForwardDynamicsModel(const std::string& model_file)
{
	mForwardDynamicsNet->LoadModel(model_file);
}


void cBaseControllerCaclaFD::BuildForwardDynamicsInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int actor_output_size = GetNumOptParams();
	Eigen::VectorXd out_offset_action = Eigen::VectorXd::Zero(actor_output_size);
	Eigen::VectorXd out_scale_action = Eigen::VectorXd::Zero(actor_output_size);
	BuildNNInputOffsetScale(out_offset, out_scale);

	Eigen::VectorXd out_offset_ = Eigen::VectorXd::Zero(out_offset.rows() + actor_output_size);
	Eigen::VectorXd out_scale_ = Eigen::VectorXd::Zero(out_scale.rows() + actor_output_size);

	BuildActorOutputOffsetScale(out_offset_action, out_scale_action);
	out_offset_.segment(0, out_offset.rows()) = out_offset;
	out_offset_.segment(out_offset.rows(), out_offset_action.rows()) = out_offset_action;
	out_scale_.segment(0, out_scale.rows()) = out_scale;
	out_scale_.segment(out_scale.rows(), out_scale_action.rows()) = out_scale_action;


	out_offset = out_offset_;
	out_scale = out_scale_;
}

void cBaseControllerCaclaFD::BuildForwardDynamicsOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	// It is the same as the actor and critic input
	BuildNNInputOffsetScale(out_offset, out_scale);
}

void cBaseControllerCaclaFD::SetForwardDynamicsInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const
{
	mForwardDynamicsNet->SetInputOffsetScale(offset, scale);
}

void cBaseControllerCaclaFD::SetForwardDynamicsOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const
{
	mForwardDynamicsNet->SetOutputOffsetScale(offset, scale);
}

void cBaseControllerCaclaFD::ComputeActionGradient(Eigen::VectorXd & in_action, Eigen::VectorXd & out_action_grad)
{
	Eigen::VectorXd out_action;
	// ExploitPolicy(mPolistate, out_action);
	mNet->Eval(mPoliState, out_action);
	// assert(out_action.size() == GetPoliActionSize());
	std::cout << "Policy Action: " << in_action.transpose() << std::endl;
	std::cout << "Policy Action Other: " << out_action.transpose() << std::endl;
	Eigen::VectorXd current_state = Eigen::VectorXd::Ones(mPoliState.rows() + GetPoliActionSize());
	Eigen::VectorXd next_state = Eigen::VectorXd::Ones(mPoliState.rows());
	current_state.segment(0, mPoliState.rows()) = mPoliState;
	current_state.segment(mPoliState.rows(), GetPoliActionSize()) = in_action;
	std::cout << "Current State: " << current_state.transpose() << std::endl;
	mForwardDynamicsNet->Eval(current_state, next_state);
	std::cout << "Next State: " << next_state.transpose() << std::endl;
	Eigen::VectorXd value;
	mCriticNet->Eval(next_state, value);
	std::cout << "Value for state: " << value.transpose() << std::endl;
	Eigen::VectorXd next_state_grad;
	Eigen::VectorXd diff = mExpParams.mNoise * Eigen::VectorXd::Ones(value.size());
	mCriticNet->Backward(diff, next_state_grad);
	std::cout << "next state grad length: " << next_state_grad.rows() << " next state grad: " << next_state_grad.transpose() << std::endl;
	Eigen::VectorXd poly_state_grad = Eigen::VectorXd::Ones(mPoliState.rows() + GetPoliActionSize());
	mForwardDynamicsNet->Backward(next_state_grad, poly_state_grad);
	std::cout << "poly state grad length: " << poly_state_grad.rows() << " poly state grad: " << poly_state_grad.transpose() << std::endl;

	out_action_grad = poly_state_grad.segment(mPoliState.rows(), GetPoliActionSize());
	std::cout << "expNoise: " << mExpParams.mNoise << " action grad length: " << out_action_grad.rows() << std::endl;
	std::cout << " action grad: " << out_action_grad.transpose() << std::endl;

}

void cBaseControllerCaclaFD::ApplyExpNoise(tAction& out_action, Eigen::VectorXd& in_action_params)
{
	int num_params = GetNumParams();
	int num_opt_params = GetNumOptParams();
	Eigen::VectorXd out_action_grad;
	ComputeActionGradient(in_action_params, out_action_grad);

	assert(out_action_grad.size() == num_opt_params);

	// for debugging
	Eigen::VectorXd exp_noise = Eigen::VectorXd::Zero(num_opt_params);

	int opt_idx = 0;
	for (int i = 0; i < num_params; ++i)
	{
		if (IsOptParam(i))
		{
			// double noise = cMathUtil::RandDoubleNorm(0, mExpParams.mNoise);
			double noise = out_action_grad[opt_idx];

			out_action.mParams[i] -= noise; // want to subtract this gradient I believe
			exp_noise[opt_idx] = noise;
			++opt_idx;
		}
	}
}

void cBaseControllerCaclaFD::ExploitPolicy(const Eigen::VectorXd& state, tAction& out_action, Eigen::VectorXd& out_params)
{
	// Eigen::VectorXd opt_params;
	mNet->Eval(state, out_params);
	assert(out_params.size() == GetPoliActionSize());

	out_action.mID = gInvalidIdx;
	out_action.mParams = mCurrAction.mParams; // This is the params for an action, not the same as a network output.

	SetOptParams(out_params, out_action.mParams);

#if defined (ENABLE_DEBUG_PRINT)
	DebugPrintAction(out_action);
	printf("\n");
#endif
}

void cBaseControllerCaclaFD::ExploreAction(Eigen::VectorXd& state, tAction& out_action)
{
#if defined (ENABLE_DEBUG_PRINT)
	printf("Exploring action\n");
#endif
	/*
	tAction out_action2;
	Eigen::VectorXd out_action3;
	ExploitPolicy(state, out_action2, out_action3);
	ApplyExpNoise(out_action2, out_action3); // for debugging
	*/
	double rand = cMathUtil::RandDouble();
	if (rand < mExpParams.mBaseActionRate)
	{
		BuildRandBaseAction(out_action);
	}
	else
	{
		rand = cMathUtil::RandDouble();
		if (rand > 0.5)
		{
			Eigen::VectorXd out_params;
			// mNet->Eval(state, out_params);
			ExploitPolicy(state, out_action, out_params);
			ApplyExpNoise(out_action, out_params);
		}
		else
		{
			cBaseControllerCacla::ExploitPolicy(state, out_action);
			cBaseControllerCacla::ApplyExpNoise(out_action);
		}
	}
}