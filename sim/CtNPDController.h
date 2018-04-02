#pragma once

#include "sim/CtController.h"

class cCtNPDController : public virtual cCtController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cCtNPDController();
	virtual ~cCtNPDController();

	virtual int GetPoliStateSize() const;
	virtual void BuildNNInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;

protected:

	virtual int GetPDStateOffset() const;
	virtual int GetPDStateSize() const;
	virtual void BuildPoliState(Eigen::VectorXd& out_state) const;
	virtual void BuildPDState(Eigen::VectorXd& out_state) const;

	virtual void BuildPDStateOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
};