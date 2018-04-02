#pragma once
#include "NeuralNetLearner.h"

struct cRNNLearner : public cNeuralNetLearner
{
public:
	cRNNLearner(const std::shared_ptr<cNeuralNetTrainer>& trainer);
	virtual ~cRNNLearner();

	virtual void Init();
	virtual void Reset();

protected:
	int mPrevID;

	virtual void AddTuples(const std::vector<tExpTuple>& tuples);
};