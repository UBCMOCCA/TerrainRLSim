#include "ScenarioVelCtrl.h"

#include <memory>
#include <ctime>
#include "util/FileUtil.h"

cScenarioVelCtrl::cScenarioVelCtrl()
{
	mCommandMaxTime = 5;
	ResetParams();
}

cScenarioVelCtrl::~cScenarioVelCtrl()
{
}

void cScenarioVelCtrl::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioTrackMotion::ParseArgs(parser);
	parser->ParseDoubleArray("target_vel_x", mTargetVels);
	parser->ParseDouble("vel_ctrl_cmd_max_time", mCommandMaxTime);
}

void cScenarioVelCtrl::Init()
{
	cScenarioTrackMotion::Init();
	ResetParams();
}

void cScenarioVelCtrl::Reset()
{
	cScenarioTrackMotion::Reset();
	ResetParams();
}

void cScenarioVelCtrl::Clear()
{
	cScenarioTrackMotion::Clear();
	ResetParams();
}

void cScenarioVelCtrl::Update(double time_elapsed)
{
	cScenarioTrackMotion::Update(time_elapsed);
	UpdateCommand(time_elapsed);
	CalcTargetVelErr();
}


std::string cScenarioVelCtrl::GetName() const
{
	return "Velocity Control";
}

void cScenarioVelCtrl::ResetParams()
{
	mCurrTargetIdx = 0;
	mCommandTimer = 0;
}

bool cScenarioVelCtrl::BuildController(std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = cScenarioSimChar::BuildController(out_ctrl);
	
	if (mBuildCtrlFromPose)
	{
		BuildCtrlParamsFromPose(mTargetCtrlID, out_ctrl);
	}
	return succ;
}

void cScenarioVelCtrl::UpdateCommand(double time_step)
{
	mCommandTimer -= time_step;
	if (mCommandTimer <= 0)
	{
		int action_id = mTargetActions[mCurrTargetIdx];
		CommandAction(action_id);
		mCurrTargetIdx = (mCurrTargetIdx + 1) % static_cast<int>(mTargetActions.size());
		mCommandTimer = mCommandMaxTime;

#if defined (ENABLE_DEBUG_PRINT)
		printf("Action: %i\n", action_id);
#endif
	}
}

double cScenarioVelCtrl::CalcTargetVelErr() const
{
	double vel_err = 0;
	int vel_idx = mCurrTargetIdx - 1;
	if (vel_idx < 0)
	{
		vel_idx = static_cast<int>(mTargetActions.size()) - 1;
	}

	if (vel_idx >= static_cast<int>(mTargetVels.size()))
	{
		vel_err = cScenarioTrackMotion::CalcTargetVelErr();
	}
	else
	{
		tVector vel = mChar->CalcCOMVel();

		double tar_vel = mTargetVels[vel_idx];
		vel_err = tar_vel - vel[0];
		vel_err *= vel_err;
	}
	return vel_err;
}