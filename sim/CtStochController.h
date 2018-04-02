#pragma once

#include "sim/CtController.h"

class cCtStochController : public virtual cCtController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cCtStochController();
	virtual ~cCtStochController();

	virtual int GetPoliStateSize() const;
	virtual int GetPoliActionSize() const;
	
	virtual void BuildNNInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const;
	virtual void BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void GetPoliActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;

	virtual int GetNumNoiseUnits() const;
	virtual int GetNoiseStateOffset() const;

protected:

	virtual void BuildPoliState(Eigen::VectorXd& out_state) const;

	virtual void ExploreAction(Eigen::VectorXd& state, tAction& out_action);
	virtual void ApplyExpNoiseInternal(Eigen::VectorXd& out_state) const;
	
	virtual int GetNoiseStateSize() const;
	virtual int GetNoiseActionOffset() const;
	virtual int GetNoiseActionSize() const;
	virtual void BuildActionNoiseBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;
};