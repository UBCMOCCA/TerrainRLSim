#include "sim/CtTargetTerrController.h"

const int gGroundSampleRes = 32;

cCtTargetTerrController::cCtTargetTerrController() : cCtPDPhaseTargetController()
{
	mViewDist = 10;
	mViewDistMin = -1;
}

cCtTargetTerrController::~cCtTargetTerrController()
{
}

int cCtTargetTerrController::GetNumGroundSamples() const
{
	return gGroundSampleRes * gGroundSampleRes;
}

int cCtTargetTerrController::GetGroundSampleRes() const
{
	return gGroundSampleRes;
}

tVector cCtTargetTerrController::CalcGroundSamplePos(int s) const
{
    return cCtController::CalcGroundSamplePos(s);
}
