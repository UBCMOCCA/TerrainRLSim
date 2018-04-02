#include "MonopedHopperControllerCacla.h"

cMonopedHopperControllerCacla::cMonopedHopperControllerCacla() : cTerrainRLCharController(),
											cMonopedHopperController(),
											cBaseControllerCacla()
{
	mExpParams.mBaseActionRate = 0.2;
	mExpParams.mNoise = 0.25;
}

cMonopedHopperControllerCacla::~cMonopedHopperControllerCacla()
{
}

void cMonopedHopperControllerCacla::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cBaseControllerCacla::Init(character);
	cMonopedHopperController::Init(character, gravity, param_file);
}

void cMonopedHopperControllerCacla::UpdateAction()
{
	cMonopedHopperController::UpdateAction();
#if defined(ENABLE_DEBUG_VISUALIZATION)
	RecordVal();
#endif // ENABLE_DEBUG_VISUALIZATION
}

bool cMonopedHopperControllerCacla::IsCurrActionCyclic() const
{
	return false;
}