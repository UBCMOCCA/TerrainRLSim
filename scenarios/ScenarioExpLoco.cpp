#include "ScenarioExpLoco.h"
#include "sim/SimCharSoftFall.h"

double cScenarioExpLoco::CalcReward() const
{
	double vel_w = 0.4;
	double root_rot_w = 0.6;

	double total_w = vel_w + root_rot_w;
	vel_w /= total_w;
	root_rot_w /= total_w;

	const double vel_scale = 1;
	const double err_scale = 1;

	double reward = 0;

	double time_elapsed = mTime - mPrevTime;
	if (time_elapsed > 0)
	{
		bool fallen = HasFallen();
		if (!fallen)
		{
			tVector com_vel = mChar->CalcCOMVel();
			double vel_err = mTargetVel[0] - com_vel[0];
			vel_err *= vel_err;

			tQuaternion tar_root_rot = tQuaternion(1, 0, 0, 0);
			tQuaternion root_rot = mChar->GetRootRotation();
			double root_rot_err = cMathUtil::QuatDiffTheta(tar_root_rot, root_rot);

			double vel_reward = std::exp(-err_scale * vel_scale * vel_err);
			double root_rot_reward = std::max(0.0, std::cos(root_rot_err));

			reward = vel_w * vel_reward + root_rot_w * root_rot_reward;
		}
	}

	return 0;
}

cScenarioExpLoco::cScenarioExpLoco()
{
	mTargetVel = tVector(1, 0, 0, 0);
	mCharParams.mEnableFallDist = false;
	mCharParams.mEnableSoftContact = false;
}

cScenarioExpLoco::~cScenarioExpLoco()
{
}

void cScenarioExpLoco::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioExpCacla::ParseArgs(parser);

	parser->ParseDouble("target_vel_x", mTargetVel[0]);
}

std::string cScenarioExpLoco::GetName() const
{
	return "Exploration Locomotion";
}