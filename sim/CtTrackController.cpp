#include "CtTrackController.h"
#include "sim/SimCharacter.h"

cCtTrackController::cCtTrackController() : cCtController()
{
}

cCtTrackController::~cCtTrackController()
{
}

void cCtTrackController::Init(cSimCharacter* character)
{
	cCtController::Init(character);
	InitTargetPoseVel();
}

int cCtTrackController::GetPoliStateSize() const
{
	int state_size = cCtController::GetPoliStateSize();
	int tar_state_size = GetTargetStateSize();
	state_size += tar_state_size;
	return state_size;
}

void cCtTrackController::SetTargetPoseVel(const Eigen::VectorXd& tar_pose, const Eigen::VectorXd& tar_vel)
{
	assert(tar_pose.size() == mChar->GetNumDof());
	assert(tar_vel.size() == mChar->GetNumDof());
	mTargetPose = tar_pose;
	mTargetVel = tar_vel;
}

const Eigen::VectorXd& cCtTrackController::GetTargetPose() const
{
	return mTargetPose;
}

const Eigen::VectorXd& cCtTrackController::GetTargetVel() const
{
	return mTargetVel;
}

void cCtTrackController::InitTargetPoseVel()
{
	mTargetPose = mChar->GetPose();
	mTargetVel = mChar->GetVel();
}

int cCtTrackController::GetTargetStateOffset() const
{
	return cCtController::GetPoliStateSize();
}

int cCtTrackController::GetTargetStateSize() const
{
	int pos_dim = GetPosFeatureDim();
	return 2 * mChar->GetNumBodyParts() * pos_dim + 1; // + 1 for root pos y
}

void cCtTrackController::BuildPoliState(Eigen::VectorXd& out_state) const
{
	Eigen::VectorXd tar_state;
	cCtController::BuildPoliState(out_state);
	BuildTargetState(tar_state);

	int tar_offset = GetTargetStateOffset();
	int tar_size = GetTargetStateSize();
	out_state.segment(tar_offset, tar_size) = tar_state;
}

void cCtTrackController::BuildTargetState(Eigen::VectorXd& out_state) const
{
	out_state.resize(GetTargetStateSize());

	const auto& joint_mat = mChar->GetJointMat();
	const auto& body_defs = mChar->GetBodyDefs();
	tVector tar_root_pos = cKinTree::GetRootPos(joint_mat, mTargetPose);
	tMatrix origin_trans = cKinTree::BuildOriginTrans(joint_mat, mTargetPose);

	tar_root_pos[3] = 1;
	tar_root_pos = origin_trans * tar_root_pos;
	tar_root_pos[3] = 0;

	out_state[0] = tar_root_pos[1];

	int idx = 1;
	int pos_dim = GetPosFeatureDim();
	int num_parts = mChar->GetNumBodyParts();
	for (int i = 0; i < num_parts; ++i)
	{
		tVector curr_pos = cKinTree::CalcBodyPartPos(joint_mat, body_defs, mTargetPose, i);
		curr_pos[3] = 1;
		curr_pos = origin_trans * curr_pos;
		curr_pos[3] = 0;

		curr_pos -= tar_root_pos;
		out_state.segment(idx, pos_dim) = curr_pos.segment(0, pos_dim);
		idx += pos_dim;
	}

	for (int i = 0; i < num_parts; ++i)
	{
		tVector curr_vel = cKinTree::CalcBodyPartVel(joint_mat, body_defs, mTargetPose, mTargetVel, i);
		curr_vel = origin_trans * curr_vel;
		out_state.segment(idx, pos_dim) = curr_vel.segment(0, pos_dim);
		idx += pos_dim;
	}
}