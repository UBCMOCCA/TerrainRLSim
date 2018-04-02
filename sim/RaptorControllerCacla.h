#pragma once

#include "sim/BaseControllerCacla.h"
#include "sim/RaptorController.h"

class cRaptorControllerCacla : public virtual cRaptorController, public virtual cBaseControllerCacla
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cRaptorControllerCacla();
	virtual ~cRaptorControllerCacla();

	virtual void Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file);

protected:
	virtual void UpdateAction();
	virtual bool IsCurrActionCyclic() const;
};