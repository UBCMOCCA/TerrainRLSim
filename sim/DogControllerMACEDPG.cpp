#include "DogControllerMACEDPG.h"

cDogControllerMACEDPG::cDogControllerMACEDPG() : cTerrainRLCharController(),
												cDogControllerDPG(),
												cBaseControllerMACEDPG()
{
}

cDogControllerMACEDPG::~cDogControllerMACEDPG()
{
}

void cDogControllerMACEDPG::Reset()
{
	cBaseControllerMACEDPG::Reset();
	cDogControllerDPG::Reset();
}

void cDogControllerMACEDPG::UpdateAction()
{
	cBaseControllerMACEDPG::UpdateAction();
	cDogControllerDPG::UpdateAction();

#if defined (ENABLE_DEBUG_PRINT)
	int state_size = GetPoliStateSize();
	int action_size = GetPoliActionSize();

	Eigen::VectorXd x;
	Eigen::VectorXd y;
	BuildCriticInput(x);

	auto& critic = GetCritic();
	critic->Eval(x, y);
	y[0] = 0.01;
	critic->Backward(y, x);

	Eigen::VectorXd dpg = x.segment(state_size, action_size);
	std::string json = BuildOptParamsJson(dpg);
	printf("DPG: \n %s\n", json.c_str());
#endif
}

void cDogControllerMACEDPG::BuildBaseAction(int action_id, tAction& out_action) const
{
	cDogControllerDPG::BuildBaseAction(action_id, out_action);
	out_action.mID = AssignFragID(action_id);
}

void cDogControllerMACEDPG::ProcessCommand(tAction& out_action)
{
	cBaseControllerMACEDPG::ProcessCommand(out_action);
	cDogControllerDPG::ProcessCommand(out_action);
}

int cDogControllerMACEDPG::AssignFragID(int a_id) const
{
	return cMathUtil::RandInt(0, GetNumActionFrags());
}

void cDogControllerMACEDPG::BuildActorBias(int a_id, Eigen::VectorXd& out_bias) const
{
	int num_ctrl_params = static_cast<int>(mCtrlParams.size());
	int ctrl_idx = a_id % num_ctrl_params;
	const Eigen::VectorXd& ctrl_params = GetCtrlParams(ctrl_idx);
	GetOptParams(ctrl_params, out_bias);
}

void cDogControllerMACEDPG::ExploitPolicy(const Eigen::VectorXd& state, tAction& out_action)
{
	cBaseControllerMACEDPG::ExploitPolicy(state, out_action);
}