#pragma once

#include "sim/CtPDController.h"
#include "sim/CtQController.h"

class cCtQPDController : public virtual cCtPDController, public virtual cCtQController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cCtQPDController();
	virtual ~cCtQPDController();

	virtual void Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file);
	
protected:
};