#pragma once

#include "sim/CtController.h"

class cCtQController : public virtual cCtController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cCtQController();
	virtual ~cCtQController();

	virtual void Init(cSimCharacter* character);

	virtual int GetCriticInputSize() const;
	virtual void BuildCriticInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;

protected:
	Eigen::MatrixXd mCriticInputBatchBuffer;
	Eigen::MatrixXd mCriticOutputBatchBuffer;
	Eigen::VectorXd mBoltzVals;

	virtual void InitBatchBuffers();

	virtual void BuildCriticInput(Eigen::VectorXd& out_x) const;
	virtual void ExploreAction(Eigen::VectorXd& state, tAction& out_action);
	virtual void ExploreActionBoltz(Eigen::VectorXd& state, tAction& out_action);
};