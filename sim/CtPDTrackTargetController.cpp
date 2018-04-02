#include "CtPDTrackTargetController.h"

cCtPDTrackTargetController::cCtPDTrackTargetController() : cCtController(),
											   cCtTargetController(),
											   cCtPDTrackController()
{
}

cCtPDTrackTargetController::~cCtPDTrackTargetController()
{
}

void cCtPDTrackTargetController::Init(cSimCharacter* character)
{
	assert(false); // do not use
	cCtTargetController::Init(character);
}

void cCtPDTrackTargetController::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cCtTargetController::Init(character);
	cCtPDTrackController::Init(character, gravity, param_file);
}

int cCtPDTrackTargetController::GetPoliStateSize() const
{
	int state_size = cCtPDTrackController::GetPoliStateSize();
	int tar_pos_size = GetTargetPosStateSize();
	state_size += tar_pos_size;
	return state_size;
}

int cCtPDTrackTargetController::GetTargetPosStateOffset() const
{
	return cCtPDTrackController::GetPoliStateSize();
}

void cCtPDTrackTargetController::BuildPoliState(Eigen::VectorXd& out_state) const
{
	Eigen::VectorXd tar_pos;
	cCtPDTrackController::BuildPoliState(out_state);
	BuildTargetPosState(tar_pos);

	int tar_offset = GetTargetPosStateOffset();
	int tar_size = GetTargetPosStateSize();
	out_state.segment(tar_offset, tar_size) = tar_pos;
}