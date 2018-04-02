#include "CtNPDTrackController.h"
#include "sim/SimCharacter.h"

cCtNPDTrackController::cCtNPDTrackController() : cCtController(),
											   cCtTrackController(),
											   cCtNPDController()
{
}

cCtNPDTrackController::~cCtNPDTrackController()
{
}

void cCtNPDTrackController::Init(cSimCharacter* character)
{
	cCtTrackController::Init(character);
	cCtNPDController::Init(character);
}

int cCtNPDTrackController::GetPoliStateSize() const
{
	int state_size = cCtTrackController::GetPoliStateSize();
	int pd_state_size = GetPDStateSize();
	state_size += pd_state_size;
	return state_size;
}

int cCtNPDTrackController::GetPDStateOffset() const
{
	return cCtTrackController::GetPoliStateSize();
}

void cCtNPDTrackController::BuildPoliState(Eigen::VectorXd& out_state) const
{
	Eigen::VectorXd pd_state;
	cCtTrackController::BuildPoliState(out_state);
	BuildPDState(pd_state);

	int pd_offset = GetPDStateOffset();
	int pd_size = GetPDStateSize();
	out_state.segment(pd_offset, pd_size) = pd_state;
}
