#include "DogControllerCacla.h"

cDogControllerCacla::cDogControllerCacla() : cTerrainRLCharController(),
											cDogController(),
											cBaseControllerCacla()
{
	mExpParams.mBaseActionRate = 0.2;
	mExpParams.mNoise = 0.2;
}

cDogControllerCacla::~cDogControllerCacla()
{
}

void cDogControllerCacla::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cBaseControllerCacla::Init(character);
	cDogController::Init(character, gravity, param_file);
}

bool cDogControllerCacla::IsCurrActionCyclic() const
{
	return false;
}

void cDogControllerCacla::UpdateAction()
{
	cDogController::UpdateAction();
#if defined(ENABLE_DEBUG_VISUALIZATION)
	RecordVal();
#endif // ENABLE_DEBUG_VISUALIZATION
}