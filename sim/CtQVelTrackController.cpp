#include "CtQVelTrackController.h"

cCtQVelTrackController::cCtQVelTrackController() : cCtQController(),
											   cCtQTrackController(),
											   cCtQVelController()
{
}

cCtQVelTrackController::~cCtQVelTrackController()
{
}

void cCtQVelTrackController::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cCtQTrackController::Init(character);
	cCtQVelController::Init(character, gravity, param_file);
}