#pragma once

#include "sim/DogControllerDPG.h"
#include "sim/BaseControllerMACEDPG.h"

class cDogControllerMACEDPG : public virtual cDogControllerDPG, public virtual cBaseControllerMACEDPG
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cDogControllerMACEDPG();
	virtual ~cDogControllerMACEDPG();

	virtual void Reset();

protected:

	virtual void UpdateAction();
	virtual void BuildBaseAction(int action_id, tAction& out_action) const;
	virtual void ProcessCommand(tAction& out_action);
	virtual int AssignFragID(int a_id) const;

	virtual void BuildActorBias(int a_id, Eigen::VectorXd& out_bias) const;
	virtual void ExploitPolicy(const Eigen::VectorXd& state, tAction& out_action);
};