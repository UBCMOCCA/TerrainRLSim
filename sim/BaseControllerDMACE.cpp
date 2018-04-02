#include "BaseControllerDMACE.h"

cBaseControllerDMACE::cBaseControllerDMACE() : cBaseControllerMACE()
{
}

cBaseControllerDMACE::~cBaseControllerDMACE()
{
}

bool cBaseControllerDMACE::ValidCritic() const
{
	return mCriticNet->HasNet();
}

void cBaseControllerDMACE::Init(cSimCharacter* character)
{
	cBaseControllerMACE::Init(character);
	BuildCriticNet(mCriticNet);
}

bool cBaseControllerDMACE::LoadCriticNet(const std::string& net_file)
{
	bool succ = true;
	mCriticNet->Clear();
	mCriticNet->LoadNet(net_file);

	int input_size = mCriticNet->GetInputSize();
	int output_size = mCriticNet->GetOutputSize();
	int critic_input_size = GetCriticInputSize();
	int critic_output_size = GetCriticOutputSize();

	if (input_size != critic_input_size)
	{
		printf("Network input dimension does not match expected input size (%i (network) vs %i (controller) ).\n", input_size, critic_input_size);
		succ = false;
	}

	if (output_size != critic_output_size)
	{
		printf("Network output dimension does not match expected output size (%i (network) vs %i (controller) ).\n", output_size, critic_output_size);
		succ = false;
	}

	if (succ)
	{
		Eigen::VectorXd input_offset;
		Eigen::VectorXd input_scale;
		BuildCriticInputOffsetScale(input_offset, input_scale);
		SetCriticInputOffsetScale(input_offset, input_scale);

		Eigen::VectorXd output_offset;
		Eigen::VectorXd output_scale;
		BuildCriticOutputOffsetScale(output_offset, output_scale);
		SetCriticOutputOffsetScale(output_offset, output_scale);
	}
	else
	{
		mCriticNet->Clear();
		assert(false);
	}

	return succ;
}

void cBaseControllerDMACE::LoadCriticModel(const std::string& model_file)
{
	mCriticNet->LoadModel(model_file);
}

void cBaseControllerDMACE::CopyNet(const cNeuralNet& net)
{
	CopyActorNet(net);
}

void cBaseControllerDMACE::CopyActorNet(const cNeuralNet& net)
{
	mNet->CopyModel(net);
}

void cBaseControllerDMACE::CopyCriticNet(const cNeuralNet& net)
{
	mCriticNet->CopyModel(net);
}

void cBaseControllerDMACE::BuildNNInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	cBaseControllerMACE::BuildNNInputOffsetScale(out_offset, out_scale);
}

void cBaseControllerDMACE::BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	BuildActorOutputOffsetScale(out_offset, out_scale);
}

void cBaseControllerDMACE::SetNNInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const
{
	SetActorInputOffsetScale(offset, scale);
}

void cBaseControllerDMACE::SetNNOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const
{
	SetActorOutputOffsetScale(offset, scale);
}

void cBaseControllerDMACE::BuildActorInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	BuildNNInputOffsetScale(out_offset, out_scale);
}

void cBaseControllerDMACE::BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	cBaseControllerMACE::BuildNNOutputOffsetScale(out_offset, out_scale);
	out_offset.segment(0, mNumActionFrags).setZero();
	out_scale.segment(0, mNumActionFrags).setOnes();
}

void cBaseControllerDMACE::SetActorInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const
{
	cBaseControllerMACE::SetNNInputOffsetScale(offset, scale);
}

void cBaseControllerDMACE::SetActorOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const
{
	cBaseControllerMACE::SetNNOutputOffsetScale(offset, scale);
}

void cBaseControllerDMACE::BuildCriticInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	BuildNNInputOffsetScale(out_offset, out_scale);
}

void cBaseControllerDMACE::BuildCriticOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int output_size = 1;
	out_offset = Eigen::VectorXd::Zero(output_size);
	out_scale = Eigen::VectorXd::Ones(output_size);
}

void cBaseControllerDMACE::SetCriticInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const
{
	mCriticNet->SetInputOffsetScale(offset, scale);
}

void cBaseControllerDMACE::SetCriticOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const
{
	mCriticNet->SetOutputOffsetScale(offset, scale);
}


const std::unique_ptr<cNeuralNet>& cBaseControllerDMACE::GetActor() const
{
	return mNet;
}

const std::unique_ptr<cNeuralNet>& cBaseControllerDMACE::GetCritic() const
{
	return mCriticNet;
}

int cBaseControllerDMACE::GetActorInputSize() const
{
	return GetPoliStateSize();
}

int cBaseControllerDMACE::GetActorOutputSize() const
{
	return GetPoliActionSize();
}

int cBaseControllerDMACE::GetCriticInputSize() const
{
	return GetPoliStateSize();
}

int cBaseControllerDMACE::GetCriticOutputSize() const
{
	return 1;
}

void cBaseControllerDMACE::BuildCriticNet(std::unique_ptr<cNeuralNet>& out_net) const
{
	out_net = std::unique_ptr<cNeuralNet>(new cNeuralNet());
}

void cBaseControllerDMACE::RecordVal(double val)
{
	if (ValidCritic())
	{
#if defined(ENABLE_DEBUG_VISUALIZATION)
		Eigen::VectorXd val;
		Eigen::VectorXd critic_x;
		BuildCriticInput(critic_x);
		mCriticNet->Eval(critic_x, val);

		double v = val[0];
		mPoliValLog.Add(v);

#if defined (ENABLE_DEBUG_PRINT)
		printf("Value: %.3f\n", v);
#endif

#endif
	}
}

void cBaseControllerDMACE::BuildCriticInput(Eigen::VectorXd& out_x) const
{
	out_x = mPoliState;
}

#if defined(ENABLE_DEBUG_VISUALIZATION)
void cBaseControllerDMACE::GetVisActionValues(Eigen::VectorXd& out_vals) const
{
	cBaseControllerMACE::GetVisActionValues(out_vals);
	if (out_vals.size() > 0)
	{
		out_vals = out_vals * 0.5 + 0.5 * Eigen::VectorXd::Ones(out_vals.size()); // values should be between [-1, 1]
	}
}
#endif
