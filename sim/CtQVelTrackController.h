#pragma once
#include "sim/CtQTrackController.h"
#include "sim/CtQVelController.h"

class cCtQVelTrackController : public virtual cCtQTrackController, public virtual cCtQVelController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cCtQVelTrackController();
	virtual ~cCtQVelTrackController();

	virtual void Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file);

protected:

};