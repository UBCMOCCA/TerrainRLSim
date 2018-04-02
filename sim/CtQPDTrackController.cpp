#include "CtQPDTrackController.h"

cCtQPDTrackController::cCtQPDTrackController() : cCtQController(),
											   cCtQTrackController(),
											   cCtQPDController()
{
}

cCtQPDTrackController::~cCtQPDTrackController()
{
}

void cCtQPDTrackController::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cCtQTrackController::Init(character);
	cCtQPDController::Init(character, gravity, param_file);
}