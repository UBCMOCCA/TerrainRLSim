#include "CtMTUTrackController.h"

cCtMTUTrackController::cCtMTUTrackController() : cCtController(),
											   cCtTrackController(),
											   cCtMTUController()
{
}

cCtMTUTrackController::~cCtMTUTrackController()
{
}

void cCtMTUTrackController::Init(cSimCharacter* character, const std::string& param_file)
{
	cCtTrackController::Init(character);
	cCtMTUController::Init(character, param_file);
}

int cCtMTUTrackController::GetPoliStateSize() const
{
	int state_size = cCtMTUController::GetPoliStateSize();
	int tar_state_size = GetTargetStateSize();
	state_size += tar_state_size;
	return state_size;
}

int cCtMTUTrackController::GetTargetStateOffset() const
{
	return cCtMTUController::GetPoliStateSize();
}

void cCtMTUTrackController::BuildPoliState(Eigen::VectorXd& out_state) const
{
	Eigen::VectorXd tar_state;
	cCtMTUController::BuildPoliState(out_state);
	BuildTargetState(tar_state);

	int tar_offset = GetTargetStateOffset();
	int tar_size = GetTargetStateSize();
	out_state.segment(tar_offset, tar_size) = tar_state;
}