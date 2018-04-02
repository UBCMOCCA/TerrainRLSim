/*
 * AsyncCaclaDVTrainer.h
 *
 *  Created on: Oct 26, 2016
 *      Author: Glen
 */

#ifndef LEARNING_ASYNCCACLADVTRAINER_H_
#define LEARNING_ASYNCCACLADVTRAINER_H_

#include "learning/AsyncCaclaTrainer.h"

class cAsyncCaclaDVTrainer : public cAsyncCaclaTrainer 
{
public:
	cAsyncCaclaDVTrainer();
	virtual ~cAsyncCaclaDVTrainer();

protected:
	virtual void BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const;

};

#endif /* LEARNING_ASYNCCACLADVTRAINER_H_ */
