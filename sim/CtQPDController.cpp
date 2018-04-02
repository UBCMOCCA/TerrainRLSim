#include "CtQPDController.h"

cCtQPDController::cCtQPDController() : cCtController(),
										cCtPDController(),
										cCtQController()
{
}

cCtQPDController::~cCtQPDController()
{
}

void cCtQPDController::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cCtQController::Init(character);
	cCtPDController::Init(character, gravity, param_file);
}