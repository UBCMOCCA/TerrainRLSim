#include "CtNPDController.h"
#include "sim/SimCharacter.h"

cCtNPDController::cCtNPDController()
{
}

cCtNPDController::~cCtNPDController()
{
}

int cCtNPDController::GetPoliStateSize() const
{
	int state_size = cCtController::GetPoliStateSize();
	int pd_state_size = GetPDStateSize();
	state_size += pd_state_size;
	return state_size;
}

void cCtNPDController::BuildNNInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	cCtController::BuildNNInputOffsetScale(out_offset, out_scale);

	Eigen::VectorXd pd_offset;
	Eigen::VectorXd pd_scale;
	BuildPDStateOffsetScale(pd_offset, pd_scale);

	int pd_state_offset = GetPDStateOffset();
	int pd_state_size = GetPDStateSize();
	out_offset.segment(pd_state_offset, pd_state_size) = pd_offset;
	out_scale.segment(pd_state_offset, pd_state_size) = pd_scale;
}


int cCtNPDController::GetPDStateOffset() const
{
	return cCtController::GetPoliStateSize();
}

int cCtNPDController::GetPDStateSize() const
{
	return 2 * mChar->GetNumDof();
}

void cCtNPDController::BuildPoliState(Eigen::VectorXd& out_state) const
{
	Eigen::VectorXd pd_state;
	cCtController::BuildPoliState(out_state);
	BuildPDState(pd_state);

	int pd_offset = GetPDStateOffset();
	int pd_size = GetPDStateSize();
	out_state.segment(pd_offset, pd_size) = pd_state;
}

void cCtNPDController::BuildPDState(Eigen::VectorXd& out_state) const
{
	const Eigen::VectorXd& pose = mChar->GetPose();
	const Eigen::VectorXd& vel = mChar->GetVel();
	
	int pose_size = static_cast<int>(pose.size());
	out_state.resize(GetPDStateSize());
	out_state.segment(0, pose_size) = pose;
	out_state.segment(pose_size, pose_size) = vel;
}

void cCtNPDController::BuildPDStateOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int pd_state_size = GetPDStateSize();
	int pose_size = pd_state_size / 2;

	out_offset = Eigen::VectorXd::Zero(pd_state_size);
	out_scale = Eigen::VectorXd::Ones(pd_state_size);

	double pose_offset = 0;
	double pose_scale = 1 / M_PI;
	double vel_offset = 0;
	double vel_scale = 1 / (10 * M_PI);

	out_offset.segment(0, pose_size) = pose_offset * Eigen::VectorXd::Ones(pose_size);
	out_scale.segment(0, pose_size) = pose_scale * Eigen::VectorXd::Ones(pose_size);
	out_offset.segment(pose_size, pose_size) = vel_offset * Eigen::VectorXd::Ones(pose_size);
	out_scale.segment(pose_size, pose_size) = vel_scale * Eigen::VectorXd::Ones(pose_size);
}