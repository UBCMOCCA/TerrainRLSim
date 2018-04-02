#include "CtPDTrackController.h"

cCtPDTrackController::cCtPDTrackController() : cCtController(),
											   cCtTrackController(),
											   cCtPDController()
{
}

cCtPDTrackController::~cCtPDTrackController()
{
}

void cCtPDTrackController::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cCtTrackController::Init(character);
	cCtPDController::Init(character, gravity, param_file);
}