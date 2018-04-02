/*
 * CaclaDVTrainer.h
 *
 *  Created on: Oct 25, 2016
 *      Author: Glen
 */

#ifndef LEARNING_CACLADVTRAINER_H_
#define LEARNING_CACLADVTRAINER_H_

#include "learning/CaclaTrainer.h"

class cCaclaDVTrainer : public cCaclaTrainer {
public:
	cCaclaDVTrainer();
	virtual ~cCaclaDVTrainer();

protected:
	virtual int GetTargetNetID(int net_id) const;
	virtual void UpdateCritic();
	virtual void BuildProblemY(int net_id, const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);

};

#endif /* LEARNING_CACLADVTRAINER_H_ */
