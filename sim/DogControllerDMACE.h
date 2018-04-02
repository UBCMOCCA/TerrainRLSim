#pragma once

#include "sim/DogController.h"
#include "sim/BaseControllerDMACE.h"

class cDogControllerDMACE : public virtual cDogController, public virtual cBaseControllerDMACE
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cDogControllerDMACE();
	virtual ~cDogControllerDMACE();

	virtual void Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file);
	virtual void Reset();

protected:

	virtual bool IsCurrActionCyclic() const;

	virtual void UpdateAction();
	virtual void BuildBaseAction(int action_id, tAction& out_action) const;
	virtual void ProcessCommand(tAction& out_action);
	virtual int AssignFragID(int a_id) const;

	virtual void BuildActorBias(int a_id, Eigen::VectorXd& out_bias) const;
};