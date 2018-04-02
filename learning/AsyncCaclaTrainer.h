#pragma once

#include "learning/AsyncACTrainer.h"

class cAsyncCaclaTrainer : public cAsyncACTrainer
{
public:

	cAsyncCaclaTrainer();
	virtual ~cAsyncCaclaTrainer();
	virtual void SetActionBounds(const Eigen::VectorXd& action_min, const Eigen::VectorXd& action_max);
	virtual void SetActionCovar(const Eigen::VectorXd& action_covar);

protected:
	Eigen::VectorXd mActionMin;
	Eigen::VectorXd mActionMax;
	Eigen::VectorXd mActionCovar;

	virtual void BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const;
	virtual void SetupTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer);
};