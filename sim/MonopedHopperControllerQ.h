#pragma once

#include "sim/MonopedHopperController.h"
#include "sim/BaseControllerQ.h"

class cMonopedHopperControllerQ : public virtual cMonopedHopperController, public virtual cBaseControllerQ
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cMonopedHopperControllerQ();
	virtual ~cMonopedHopperControllerQ();

protected:

};