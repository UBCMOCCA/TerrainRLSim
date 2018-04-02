#pragma once

#include "sim/CtPDController.h"

class cCtVelController : public virtual cCtPDController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cCtVelController();
	virtual ~cCtVelController();

protected:
	
	virtual void SetupPDControllers(const std::string& param_file, const tVector& gravity);
	virtual void ApplyAction(const tAction& action);
	virtual void BuildJointActionBounds(int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;
	virtual void BuildJointActionOffsetScale(int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
};