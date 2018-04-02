#pragma once
#include "sim/CtQTrackController.h"
#include "sim/CtQPDController.h"

class cCtQPDTrackController : public virtual cCtQTrackController, public virtual cCtQPDController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cCtQPDTrackController();
	virtual ~cCtQPDTrackController();

	virtual void Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file);

protected:

};