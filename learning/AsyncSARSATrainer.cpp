#include "AsyncSARSATrainer.h"
#include "SARSATrainer.h"

cAsyncSARSATrainer::cAsyncSARSATrainer()
{
}

cAsyncSARSATrainer::~cAsyncSARSATrainer()
{
}

void cAsyncSARSATrainer::BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const
{
	out_trainer = std::shared_ptr<cNeuralNetTrainer>(new cSARSATrainer());
}