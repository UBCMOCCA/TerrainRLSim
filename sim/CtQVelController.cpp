#include "CtQVelController.h"

cCtQVelController::cCtQVelController() : cCtController(),
										cCtVelController(),
										cCtQController()
{
}

cCtQVelController::~cCtQVelController()
{
}

void cCtQVelController::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cCtQController::Init(character);
	cCtVelController::Init(character, gravity, param_file);
}