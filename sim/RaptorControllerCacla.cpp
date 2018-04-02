#include "RaptorControllerCacla.h"

cRaptorControllerCacla::cRaptorControllerCacla() : cTerrainRLCharController(),
											cRaptorController(),
											cBaseControllerCacla()
{
	mExpParams.mBaseActionRate = 0.2;
	mExpParams.mNoise = 0.15;
}

cRaptorControllerCacla::~cRaptorControllerCacla()
{
}

void cRaptorControllerCacla::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cBaseControllerCacla::Init(character);
	cRaptorController::Init(character, gravity, param_file);
}

void cRaptorControllerCacla::UpdateAction()
{
	cRaptorController::UpdateAction();
#if defined(ENABLE_DEBUG_VISUALIZATION)
	RecordVal();
#endif // ENABLE_DEBUG_VISUALIZATION
}

bool cRaptorControllerCacla::IsCurrActionCyclic() const
{
	return false;
}