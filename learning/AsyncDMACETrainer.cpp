#include "AsyncDMACETrainer.h"
#include "DMACETrainer.h"

cAsyncDMACETrainer::cAsyncDMACETrainer()
{
	mNumActionFrags = 1;
	mActionFragSize = 1;
	mTemp = 1;
	mGateScale = 1;
}

cAsyncDMACETrainer::~cAsyncDMACETrainer()
{
}

void cAsyncDMACETrainer::SetNumActionFrags(int num)
{
	mNumActionFrags = num;
	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		auto dmace_trainer = std::static_pointer_cast<cDMACETrainer>(mTrainers[i]);
		dmace_trainer->SetNumActionFrags(mNumActionFrags);
	}
}

void cAsyncDMACETrainer::SetActionFragSize(int size)
{
	mActionFragSize = size;
	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		auto dmace_trainer = std::static_pointer_cast<cDMACETrainer>(mTrainers[i]);
		dmace_trainer->SetActionFragSize(mActionFragSize);
	}
}

void cAsyncDMACETrainer::SetTemp(double temp)
{
	mTemp = temp;
	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		auto dmace_trainer = std::static_pointer_cast<cDMACETrainer>(mTrainers[i]);
		dmace_trainer->SetTemp(temp);
	}
}

void cAsyncDMACETrainer::SetGateScale(double scale)
{
	mGateScale = scale;
	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		auto dmace_trainer = std::static_pointer_cast<cDMACETrainer>(mTrainers[i]);
		dmace_trainer->SetGateScale(mGateScale);
	}
}

void cAsyncDMACETrainer::BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const
{
	out_trainer = std::shared_ptr<cNeuralNetTrainer>(new cDMACETrainer());
}

void cAsyncDMACETrainer::SetupTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer)
{
	cAsyncCaclaTrainer::SetupTrainer(out_trainer);

	auto dmace_trainer = std::static_pointer_cast<cDMACETrainer>(out_trainer);
	dmace_trainer->SetNumActionFrags(mNumActionFrags);
	dmace_trainer->SetActionFragSize(mActionFragSize);
	dmace_trainer->SetTemp(mTemp);
	dmace_trainer->SetGateScale(mGateScale);
}