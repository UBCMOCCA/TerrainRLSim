#include "CtQTrackController.h"

cCtQTrackController::cCtQTrackController() : cCtQController(),
											cCtTrackController()
{
}

cCtQTrackController::~cCtQTrackController()
{
}

void cCtQTrackController::Init(cSimCharacter* character)
{
	cCtQController::Init(character);
	cCtTrackController::Init(character);
}