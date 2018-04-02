#include "ScenarioJump.h"

#include <memory>
#include <ctime>
#include "sim/SimDog.h"
#include "sim/DogController.h"
#include "util/FileUtil.h"

cScenarioJump::cScenarioJump()
{
	mRecoveryTime = 4;
	mWarmUpTime = 4;
	ResetParams();
}

cScenarioJump::~cScenarioJump()
{

}

void cScenarioJump::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioTrackMotion::ParseArgs(parser);
	parser->ParseDouble("recovery_time", mRecoveryTime);
	parser->ParseDouble("warmup_time", mWarmUpTime);
}

void cScenarioJump::Init()
{
	cScenarioTrackMotion::Init();
	ResetParams();
}

void cScenarioJump::Reset()
{
	mKinChar.SetOriginPos(tVector::Zero());
	cScenarioTrackMotion::Reset();
	ResetParams();
}

void cScenarioJump::Clear()
{
	cScenarioTrackMotion::Clear();
	ResetParams();
}

void cScenarioJump::Update(double time_elapsed)
{
	cScenarioTrackMotion::Update(time_elapsed);
}

bool cScenarioJump::JumpCompleted() const
{
	return mJumpCompleted;
}

bool cScenarioJump::Jumping() const
{
	return mJumpState == eJumpStateJump;
}

void cScenarioJump::ResetJumpCompleted()
{
	mJumpCompleted = false;
}

tVector cScenarioJump::GetJumpDist() const
{
	return mJumpEndPos - mJumpStartPos;
}

double cScenarioJump::CalcPoseThetaRelErr() const
{
	const Eigen::VectorXd& kin_pose = mKinChar.GetPose();
	const Eigen::VectorXd& sim_pose = mChar->GetPose();
	
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	double total_err = 0;
	int root_id = mChar->GetRootID();
	int num_joints = mKinChar.GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		if (j != root_id)
		{
			int offset = mChar->GetParamOffset(j);
			int size = mChar->GetParamSize(j);

			double err = 0;
			err = (kin_pose - sim_pose).segment(offset, size).squaredNorm();
			double w = mChar->GetJointDiffWeight(j);
			total_err += w * err;
		}
	}
	return total_err;
}

double cScenarioJump::CalcJumpHeightReward() const
{
	double jump_reward = 0;
	if (Jumping())
	{
		tVector com = mChar->CalcCOM();
		double h = com[1];
		jump_reward = h;
	}
	return jump_reward;
}

std::string cScenarioJump::GetName() const
{
	return "Jump";
}

void cScenarioJump::ResetParams()
{
	mJumpState = eJumpStateRecovery;
	mRecoveryTimer = mWarmUpTime;
	mCurrTargetIdx = 0;
	mJumpStartPos.setZero();
	mJumpEndPos.setZero();
	ResetJumpCompleted();
}

bool cScenarioJump::BuildController(std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = cScenarioSimChar::BuildController(out_ctrl);
	
	if (mBuildCtrlFromPose)
	{
		BuildCtrlParamsFromPose(mTargetCtrlID, out_ctrl);
	}
	return succ;
}

void cScenarioJump::UpdateJump(double time_step)
{
	if (mJumpState == eJumpStateRecovery)
	{
		mRecoveryTimer -= time_step;

		if (mRecoveryTimer <= 0)
		{
			int action_id = mTargetActions[mCurrTargetIdx];
			CommandAction(action_id);

			mJumpState = eJumpStateCommand;
			mCurrTargetIdx = (mCurrTargetIdx + 1) % static_cast<int>(mTargetActions.size());
		}
	}
	else if (mJumpState == eJumpStateCommand)
	{
		bool new_cycle = IsNewCycle();
		if (new_cycle)
		{
			mJumpState = eJumpStateJump;
			mJumpStartPos = mChar->CalcCOM();
		}
	}
	else if (mJumpState == eJumpStateJump)
	{
		bool new_cycle = IsNewCycle();
		bool stumble = mChar->HasStumbled();
		if (new_cycle || stumble)
		{
			mRecoveryTimer = mRecoveryTime;
			mJumpState = eJumpStateRecovery;
			mJumpEndPos = mChar->CalcCOM();
			mJumpCompleted = true;
		}
	}
}

void cScenarioJump::PreSubstepUpdate(double time_step)
{
	cScenarioTrackMotion::PreSubstepUpdate(time_step);
	UpdateJump(time_step);
}

bool cScenarioJump::IsNewCycle() const
{
	const auto& ctrl = mChar->GetController();
	return ctrl->IsNewCycle();
}

void cScenarioJump::UpdateKinChar(double time_step)
{
	if (mKinChar.HasMotion())
	{
		SyncKinChar();
	}
}

void cScenarioJump::SyncKinChar()
{
	const auto& ctrl = mChar->GetController();
	double phase = ctrl->CalcNormPhase();
	double motion_dur = mKinChar.GetMotionDuration();
	double kin_time = motion_dur * phase;
	mKinChar.Pose(kin_time);

	tVector sim_pos = mChar->GetRootPos();
	tVector kin_pos = mKinChar.GetRootPos();
	mKinChar.MoveOrigin(sim_pos - kin_pos);
}