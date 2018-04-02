#pragma once
#include "sim/CtTrackController.h"
#include "sim/CtVelController.h"

class cCtVelTrackController : public virtual cCtTrackController, public virtual cCtVelController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cCtVelTrackController();
	virtual ~cCtVelTrackController();

	virtual void Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file);

protected:

};