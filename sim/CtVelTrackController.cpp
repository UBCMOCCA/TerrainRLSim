#include "CtVelTrackController.h"

cCtVelTrackController::cCtVelTrackController() : cCtController(),
											   cCtTrackController(),
											   cCtVelController()
{
}

cCtVelTrackController::~cCtVelTrackController()
{
}

void cCtVelTrackController::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cCtTrackController::Init(character);
	cCtVelController::Init(character, gravity, param_file);
}