#pragma once
#include "sim/CtPhaseController.h"
#include "sim/CtVelController.h"

class cCtVelPhaseController : public virtual cCtPhaseController, public virtual cCtVelController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cCtVelPhaseController();
	virtual ~cCtVelPhaseController();

	virtual void Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file);

protected:

};