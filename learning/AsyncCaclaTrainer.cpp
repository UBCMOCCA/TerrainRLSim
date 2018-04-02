#include "AsyncCaclaTrainer.h"
#include "CaclaTrainer.h"

cAsyncCaclaTrainer::cAsyncCaclaTrainer()
{
	mActionMin = Eigen::VectorXd();
	mActionMax = Eigen::VectorXd();
}

cAsyncCaclaTrainer::~cAsyncCaclaTrainer()
{
}

void cAsyncCaclaTrainer::SetActionBounds(const Eigen::VectorXd& action_min, const Eigen::VectorXd& action_max)
{
	mActionMin = action_min;
	mActionMax = action_max;

	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		auto trainer = std::static_pointer_cast<cCaclaTrainer>(mTrainers[i]);
		trainer->SetActionBounds(mActionMin, mActionMax);
	}
}

void cAsyncCaclaTrainer::SetActionCovar(const Eigen::VectorXd& action_covar)
{
	mActionCovar = action_covar;

	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		auto trainer = std::static_pointer_cast<cCaclaTrainer>(mTrainers[i]);
		trainer->SetActionCovar(mActionCovar);
	}
}

void cAsyncCaclaTrainer::BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const
{
	out_trainer = std::shared_ptr<cNeuralNetTrainer>(new cCaclaTrainer());
}

void cAsyncCaclaTrainer::SetupTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer)
{
	cAsyncACTrainer::SetupTrainer(out_trainer);
	auto cacla_trainer = std::dynamic_pointer_cast<cCaclaTrainer>(out_trainer);
	
	if (mActionMin.size() > 0 && mActionMax.size() > 0)
	{
		cacla_trainer->SetActionBounds(mActionMin, mActionMax);
	}

	if (mActionCovar.size() > 0)
	{
		cacla_trainer->SetActionCovar(mActionCovar);
	}
}