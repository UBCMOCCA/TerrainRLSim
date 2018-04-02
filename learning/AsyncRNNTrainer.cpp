#include "AsyncRNNTrainer.h"
#include "RNNTrainer.h"

cAsyncRNNTrainer::cAsyncRNNTrainer()
{
}

cAsyncRNNTrainer::~cAsyncRNNTrainer()
{
}

void cAsyncRNNTrainer::BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const
{
	out_trainer = std::shared_ptr<cRNNTrainer>(new cRNNTrainer());
}