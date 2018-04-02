#include "CtPDPhaseTargetController.h"

cCtPDPhaseTargetController::cCtPDPhaseTargetController() : cCtController(),
											   cCtTargetController(),
											   cCtPDPhaseController()
{
}

cCtPDPhaseTargetController::~cCtPDPhaseTargetController()
{
}

void cCtPDPhaseTargetController::Init(cSimCharacter* character)
{
	assert(false); // do not use
	cCtTargetController::Init(character);
}

void cCtPDPhaseTargetController::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cCtTargetController::Init(character);
	cCtPDPhaseController::Init(character, gravity, param_file);
}

int cCtPDPhaseTargetController::GetPoliStateSize() const
{
	int state_size = cCtPDPhaseController::GetPoliStateSize();
	int tar_pos_size = GetTargetPosStateSize();
	state_size += tar_pos_size;
	return state_size;
}

int cCtPDPhaseTargetController::GetTargetPosStateOffset() const
{
	return cCtPDPhaseController::GetPoliStateSize();
}

void cCtPDPhaseTargetController::BuildPoliState(Eigen::VectorXd& out_state) const
{
	Eigen::VectorXd tar_pos;
	cCtPDPhaseController::BuildPoliState(out_state);
	BuildTargetPosState(tar_pos);

	int tar_offset = GetTargetPosStateOffset();
	int tar_size = GetTargetPosStateSize();
	out_state.segment(tar_offset, tar_size) = tar_pos;
}