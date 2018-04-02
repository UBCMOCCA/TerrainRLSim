/*
 * ACDLearner.h
 *
 *  Created on: Nov 18, 2016
 *      Author: Glen
 */

#ifndef LEARNING_ACDLEARNER_H_
#define LEARNING_ACDLEARNER_H_

#include "ACLearner.h"

class cACDLearner : public cACLearner {
public:
	cACDLearner(const std::shared_ptr<cNeuralNetTrainer>& trainer);
	virtual ~cACDLearner();

	virtual void OutputForwardDynamics(const std::string& filename) const;
	virtual void SyncNet();
	virtual bool HasForwardDynamicsNet() const;
	virtual void LoadForwardDynamicsNet(const std::string& net_file);
	virtual void LoadForwardDynamicsSolver(const std::string& solver_file);
	virtual void SetForwardDynamicsNet(cNeuralNet* net);
	virtual const cNeuralNet* GetForwardDynamicsNet() const;
	virtual void Init();

protected:
	cNeuralNet* mForwardDynamicsNet;
};

#endif /* LEARNING_ACDLEARNER_H_ */
