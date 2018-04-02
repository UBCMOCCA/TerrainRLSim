#pragma once

#include "sim/CtQController.h"
#include "sim/CtTrackController.h"

class cCtQTrackController : public virtual cCtQController, public virtual cCtTrackController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cCtQTrackController();
	virtual ~cCtQTrackController();

	virtual void Init(cSimCharacter* character);

protected:
};