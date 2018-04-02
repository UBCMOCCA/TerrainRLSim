#include "BaseControllerDPG.h"

cBaseControllerDPG::cBaseControllerDPG() : cBaseControllerCacla()
{
}

cBaseControllerDPG::~cBaseControllerDPG()
{
}

int cBaseControllerDPG::GetCriticInputSize() const
{
	return GetPoliStateSize() + GetPoliActionSize();
}

void cBaseControllerDPG::SetupDPGBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	int dpg_size = GetPoliActionSize();
	out_min = -std::numeric_limits<double>::infinity() * Eigen::VectorXd::Ones(dpg_size);
	out_max = std::numeric_limits<double>::infinity() * Eigen::VectorXd::Ones(dpg_size);
}

void cBaseControllerDPG::BuildCriticInput(Eigen::VectorXd& out_x) const
{
	int state_size = GetPoliStateSize();
	int action_size = GetPoliActionSize();

	Eigen::VectorXd action;
	RecordPoliAction(action);

	out_x.resize(GetCriticInputSize());
	assert(out_x.size() == action.size() + mPoliState.size());
	out_x.segment(0, state_size) = mPoliState;
	out_x.segment(state_size, action_size) = action;
}

void cBaseControllerDPG::ExploreAction(Eigen::VectorXd& state, tAction& out_action)
{
#if defined (ENABLE_DEBUG_PRINT)
	printf("Exploring action\n");
#endif

	double rand = cMathUtil::RandDouble();
	if (rand < mExpParams.mBaseActionRate)
	{
		BuildRandBaseAction(out_action);
		//ApplyExpNoise(out_action);
	}
	else
	{
		ExploitPolicy(state, out_action);
		ApplyExpNoise(out_action);
	}
}