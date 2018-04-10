#pragma once
#include "sim/CtPDPhaseTargetController.h"

class cCtTargetTerrController : public virtual cCtPDPhaseTargetController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cCtTargetTerrController();
	virtual ~cCtTargetTerrController();

	virtual int GetNumGroundSamples() const;
	tVector CalcGroundSamplePos(int s) const;
	
protected:

	virtual int GetGroundSampleRes() const;
};
