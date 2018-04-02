#include "AsyncDPGTrainer.h"
#include "DPGTrainer.h"

cAsyncDPGTrainer::cAsyncDPGTrainer()
{
	mQDiff = 1;
	mTargetLerp = 0.001;
	mDPGReg = 0.01;
}

cAsyncDPGTrainer::~cAsyncDPGTrainer()
{
}

void cAsyncDPGTrainer::SetQDiff(double q_diff)
{
	mQDiff = q_diff;

	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		auto dpg_trainer = std::static_pointer_cast<cDPGTrainer>(mTrainers[i]);
		dpg_trainer->SetQDiff(mQDiff);
	}
}

void cAsyncDPGTrainer::SetTargetLerp(double lerp)
{
	mTargetLerp = lerp;

	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		auto dpg_trainer = std::static_pointer_cast<cDPGTrainer>(mTrainers[i]);
		dpg_trainer->SetTargetLerp(mTargetLerp);
	}
}

void cAsyncDPGTrainer::SetDPGReg(double reg)
{
	mDPGReg = reg;

	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		auto dpg_trainer = std::static_pointer_cast<cDPGTrainer>(mTrainers[i]);
		dpg_trainer->SetDPGReg(mDPGReg);
	}
}


void cAsyncDPGTrainer::BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const
{
	out_trainer = std::shared_ptr<cDPGTrainer>(new cDPGTrainer());
}

void cAsyncDPGTrainer::SetupTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer)
{
	cAsyncCaclaTrainer::SetupTrainer(out_trainer);
	auto dpg_trainer = std::static_pointer_cast<cDPGTrainer>(out_trainer);

	dpg_trainer->SetQDiff(mQDiff);
	dpg_trainer->SetTargetLerp(mTargetLerp);
	dpg_trainer->SetDPGReg(mDPGReg);
}