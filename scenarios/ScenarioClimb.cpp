#include "ScenarioClimb.h"

#include <memory>
#include <ctime>
#include "sim/SimDog.h"
#include "sim/DogController.h"
#include "util/FileUtil.h"

// this scenario is mainly for the dog....i mean goat....
cScenarioClimb::cScenarioClimb()
{
	mLandingTargetStep.setZero();
	mLandingTargetOffset.setZero();
	ResetParams();
}

cScenarioClimb::~cScenarioClimb()
{

}

void cScenarioClimb::Init()
{
	cScenarioTrackMotion::Init();
	ResetParams();
}

void cScenarioClimb::Update(double time_elapsed)
{
	cScenarioTrackMotion::Update(time_elapsed);
	UpdateLandingPos();
}

void cScenarioClimb::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioTrackMotion::ParseArgs(parser);

	parser->ParseDouble("landing_target_step_x", mLandingTargetStep[0]);
	parser->ParseDouble("landing_target_offset_x", mLandingTargetOffset[0]);
}

void cScenarioClimb::Reset()
{
	cScenarioTrackMotion::Reset();
	ResetParams();
}

void cScenarioClimb::Clear()
{
	cScenarioTrackMotion::Clear();
	ResetParams();
}

tVector cScenarioClimb::CalcLandingTarget() const
{
	tVector tar_pos = mLandingTargetOffset;

	int num_steps = mStepCount;
	int state = mChar->GetState();

	if (state >= 2)
	{
		num_steps = mStepCount + 1;
		tar_pos += mLandingTargetStep * num_steps;
	}
	else
	{
		tar_pos = mFrontLandingPos;
	}

	double h = mGround->SampleHeight(tar_pos);
	tar_pos[1] = h;
	return tar_pos;
}

double cScenarioClimb::CalcLandingTargetErr() const
{
	// dog specific
	auto dog_ctrl = std::dynamic_pointer_cast<cDogController>(mChar->GetController());
	int state = dog_ctrl->GetState();

	double err = 1;
	if (state == cDogController::eStateBackStance
		|| state == cDogController::eStateFrontStance)
	{
		tVector tar_pos = CalcLandingTarget();

		int end_eff_id = 0;
		if (state == cDogController::eStateBackStance)
		{
			end_eff_id = cSimDog::eJointToe;
		}
		else
		{
			end_eff_id = cSimDog::eJointFinger;
		}

		tVector end_pos = mChar->GetBodyPart(end_eff_id)->GetPos();
		err *= (tar_pos - end_pos).squaredNorm();
	}

	return err;
}

double cScenarioClimb::CalcCOMErr() const
{
	tVector tar_pos = CalcLandingTarget();
	tVector com = mChar->CalcCOM();

	double target_h = tar_pos[1] + 0.4;
	double h_err = target_h - com[1];

	double err = std::max(0.0, h_err);
	err *= err;
	return err;
}

double cScenarioClimb::CalcTargetVelErr() const
{
	tVector vel = mChar->CalcCOMVel();
	double vel_err = mTargetVelX - vel[0];
	vel_err = std::max(vel_err, 0.0);
	vel_err *= vel_err;
	return vel_err;
}

std::string cScenarioClimb::GetName() const
{
	return "Climb";
}

void cScenarioClimb::ResetParams()
{
	mStepCount = 0;
	mFrontLandingPos = mCharParams.mInitPos;
}

void cScenarioClimb::UpdateKinChar(double time_step)
{
	SyncKinChar();
}

void cScenarioClimb::SyncKinChar()
{
	Eigen::VectorXd sync_pose;
	CalcSyncKinPose(sync_pose);
	mKinChar.SetPose(sync_pose);
}

void cScenarioClimb::CalcSyncKinPose(Eigen::VectorXd& out_pose) const
{
	cScenarioTrackMotion::CalcSyncKinPose(out_pose);

	const Eigen::MatrixXd& joint_mat = mKinChar.GetJointMat();
	tVector sim_pos = mChar->GetRootPos();
	tVector kin_pos = cKinTree::GetRootPos(joint_mat, out_pose);
	kin_pos[1] = sim_pos[1];
	cKinTree::SetRootPos(joint_mat, kin_pos, out_pose);
}

void cScenarioClimb::PostSubstepUpdate(double time_step)
{
	bool new_cycle = IsNewCycle();
	if (new_cycle)
	{
		tVector root_pos = mChar->GetRootPos();
		tVector tar_pos = CalcLandingTarget();
		if (root_pos[0] > tar_pos[0]
			|| std::abs((root_pos[0] - tar_pos[0])) < 0.5)
		{
			++mStepCount;
		}
	}
}

void cScenarioClimb::UpdateLandingPos()
{
	// this is for the dog
	int state = mChar->GetState();
	double phase = mChar->GetPhase();
	if (state == cDogController::eStateFrontStance && phase < 0.25)
	{
		mFrontLandingPos = mChar->GetBodyPart(cSimDog::eJointFinger)->GetPos();
	}
}