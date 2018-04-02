#include "CtVelPhaseController.h"

cCtVelPhaseController::cCtVelPhaseController() : cCtController(),
											   cCtPhaseController(),
											   cCtVelController()
{
}

cCtVelPhaseController::~cCtVelPhaseController()
{
}

void cCtVelPhaseController::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cCtPhaseController::Init(character);
	cCtVelController::Init(character, gravity, param_file);
}