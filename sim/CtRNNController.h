#pragma once

#include "sim/CtController.h"
#include "learning/RecurrentNet.h"

class cCtRNNController : public virtual cCtController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cCtRNNController();
	virtual ~cCtRNNController();

protected:

	virtual void BuildNet(std::unique_ptr<cNeuralNet>& out_net) const;
	virtual const cRecurrentNet* GetRNN() const;

	virtual void ExploitPolicy(const Eigen::VectorXd& state, tAction& out_action);
};