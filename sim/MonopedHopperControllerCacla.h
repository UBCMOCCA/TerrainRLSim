#pragma once

#include "sim/BaseControllerCacla.h"
#include "sim/MonopedHopperController.h"

class cMonopedHopperControllerCacla : public virtual cMonopedHopperController, public virtual cBaseControllerCacla
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cMonopedHopperControllerCacla();
	virtual ~cMonopedHopperControllerCacla();

	virtual void Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file);

protected:
	virtual void UpdateAction();
	virtual bool IsCurrActionCyclic() const;
};