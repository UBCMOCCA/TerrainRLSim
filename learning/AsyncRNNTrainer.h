#pragma once

#include "learning/AsyncTrainer.h"

class cAsyncRNNTrainer : public cAsyncTrainer
{
public:

	cAsyncRNNTrainer();
	virtual ~cAsyncRNNTrainer();

protected:
	
	virtual void BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const;
};