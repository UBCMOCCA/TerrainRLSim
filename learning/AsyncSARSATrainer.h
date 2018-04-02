#pragma once

#include "learning/AsyncCaclaTrainer.h"

class cAsyncSARSATrainer : public cAsyncCaclaTrainer
{
public:

	cAsyncSARSATrainer();
	virtual ~cAsyncSARSATrainer();

protected:
	
	virtual void BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const;
};