/*
 * AsyncCaclaDVTrainer.cpp
 *
 *  Created on: Oct 26, 2016
 *      Author: Glen
 */

#include "AsyncCaclaDVTrainer.h"
#include "CaclaDVTrainer.h"

cAsyncCaclaDVTrainer::cAsyncCaclaDVTrainer() : cAsyncCaclaTrainer() {
	// TODO Auto-generated constructor stub

}

cAsyncCaclaDVTrainer::~cAsyncCaclaDVTrainer() {
	// TODO Auto-generated destructor stub
}

void cAsyncCaclaDVTrainer::BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const
{
	out_trainer = std::shared_ptr<cNeuralNetTrainer>(new cCaclaDVTrainer());
}

