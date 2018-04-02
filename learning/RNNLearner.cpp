#include "RNNLearner.h"
#include "RNNTrainer.h"

cRNNLearner::cRNNLearner(const std::shared_ptr<cNeuralNetTrainer>& trainer)
	: cNeuralNetLearner(trainer)
{
	mPrevID = gInvalidIdx;
}

cRNNLearner::~cRNNLearner()
{
}

void cRNNLearner::Reset()
{
	cNeuralNetLearner::Reset();
	mPrevID = gInvalidIdx;
}

void cRNNLearner::Init()
{
	cNeuralNetLearner::Init();
	mPrevID = gInvalidIdx;
}

void cRNNLearner::AddTuples(const std::vector<tExpTuple>& tuples)
{
	cRNNTrainer* trainer = static_cast<cRNNTrainer*>(mTrainer.get());
	for (size_t i = 0; i < tuples.size(); ++i)
	{
		const tExpTuple& curr_tuple = tuples[i];
		bool is_start = curr_tuple.GetFlag(tExpTuple::eFlagStart);
		int prev_id = (is_start) ? gInvalidIdx : mPrevID;
		int new_id = trainer->AddTuple(curr_tuple, prev_id, mID);
		mPrevID = new_id;
	}
}