#pragma once

#include "sim/MonopedHopperController.h"
#include "sim/BaseControllerMACE.h"

class cMonopedHopperControllerMACE : public virtual cMonopedHopperController, public virtual cBaseControllerMACE
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cMonopedHopperControllerMACE();
	virtual ~cMonopedHopperControllerMACE();

	virtual void Reset();

protected:
	virtual bool IsCurrActionCyclic() const;

	virtual void UpdateAction();
	virtual void BuildBaseAction(int action_id, tAction& out_action) const;
	virtual void ProcessCommand(tAction& out_action);
	virtual int AssignFragID(int a_id) const;

	virtual void BuildActorBias(int a_id, Eigen::VectorXd& out_bias) const;
};