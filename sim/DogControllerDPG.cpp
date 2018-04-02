#include "DogControllerDPG.h"
#include "util/FileUtil.h"

cDogControllerDPG::cDogControllerDPG() : cTerrainRLCharController(),
											cDogControllerCacla(),
											cBaseControllerDPG()
{
	mExpParams.mBaseActionRate = 0.2;
	mExpParams.mNoise = 0.2;
}

cDogControllerDPG::~cDogControllerDPG()
{
}

void cDogControllerDPG::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cBaseControllerDPG::Init(character);
	cDogControllerCacla::Init(character, gravity, param_file);
}

void cDogControllerDPG::SetupDPGBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	int dpg_size = GetPoliActionSize();
	out_min = Eigen::VectorXd::Zero(dpg_size);
	out_max = Eigen::VectorXd::Zero(dpg_size);

	/*
	int idx = 0;
	for (int i = 0; i < GetNumParams(); ++i)
	{
		if (IsOptParam(i))
		{
			double bound_min = GetParamBoundMin(i);
			double bound_max = GetParamBoundMax(i);
			out_min[idx] = bound_min;
			out_max[idx] = bound_max;
			++idx;
		}
		assert(idx == dpg_size);
	}*
	*/

	const double bound_scale = 4;
	Eigen::VectorXd default_action;
	BuildActionOptParams(GetDefaultAction(), default_action);
	Eigen::VectorXd max_delta = Eigen::VectorXd::Zero(default_action.size());
	for (int a = 0; a < GetNumActions(); ++a)
	{
		Eigen::VectorXd curr_action;
		BuildActionOptParams(a, curr_action);
		Eigen::VectorXd delta = (curr_action - default_action).cwiseAbs();
		max_delta = max_delta.cwiseMax(delta);
	}

	out_min = default_action - bound_scale * max_delta;
	out_max = default_action + bound_scale * max_delta;
}

void cDogControllerDPG::ExploitPolicy(const Eigen::VectorXd& state, tAction& out_action)
{
	cBaseControllerDPG::ExploitPolicy(state, out_action);

#if defined (ENABLE_DEBUG_PRINT)
	int state_size = GetPoliStateSize();
	int action_size = GetPoliActionSize();

	Eigen::VectorXd x;
	Eigen::VectorXd y;
	BuildCriticInput(x);

	auto& critic = GetCritic();
	critic->Eval(x, y);
	y[0] = 1;
	critic->Backward(y, x);

	Eigen::VectorXd dpg = x.segment(state_size, action_size);
	Eigen::VectorXd action_scale = mNet->GetOutputScale().segment(0, action_size);
	dpg = dpg.cwiseQuotient(action_scale);
	dpg = dpg.cwiseQuotient(action_scale);

	std::string json = BuildOptParamsJson(dpg);
	printf("DPG: \n %s\n", json.c_str());
#endif
}