#pragma once
#include "sim/CtTrackController.h"
#include "sim/CtPDController.h"

class cCtPDTrackController : public virtual cCtTrackController, public virtual cCtPDController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cCtPDTrackController();
	virtual ~cCtPDTrackController();

	virtual void Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file);

protected:

};