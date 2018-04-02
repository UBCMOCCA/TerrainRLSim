#pragma once

#include "sim/BaseControllerDPG.h"
#include "sim/DogControllerCacla.h"

class cDogControllerDPG : public virtual cDogControllerCacla, public virtual cBaseControllerDPG
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cDogControllerDPG();
	virtual ~cDogControllerDPG();
	
	virtual void Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file);
	virtual void SetupDPGBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;

protected:

	virtual void ExploitPolicy(const Eigen::VectorXd& state, tAction& out_action);
};