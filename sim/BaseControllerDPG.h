#pragma once

#include "sim/BaseControllerCacla.h"

class cBaseControllerDPG : public virtual cBaseControllerCacla
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cBaseControllerDPG();
	virtual ~cBaseControllerDPG();

	virtual int GetCriticInputSize() const;
	virtual void SetupDPGBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;

protected:

	virtual void BuildCriticInput(Eigen::VectorXd& out_x) const;
	virtual void ExploreAction(Eigen::VectorXd& state, tAction& out_action);
};