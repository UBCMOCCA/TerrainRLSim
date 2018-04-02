#include "AsyncMACEDPGTrainer.h"
#include "MACEDPGTrainer.h"

cAsyncMACEDPGTrainer::cAsyncMACEDPGTrainer()
{
}

cAsyncMACEDPGTrainer::~cAsyncMACEDPGTrainer()
{
}

void cAsyncMACEDPGTrainer::SetNumActionFrags(int num)
{
	mNumActionFrags = num;
	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		auto mace_dpg_trainer = std::static_pointer_cast<cMACEDPGTrainer>(mTrainers[i]);
		mace_dpg_trainer->SetNumActionFrags(mNumActionFrags);
	}
}

void cAsyncMACEDPGTrainer::SetActionFragSize(int size)
{
	mActionFragSize = size;
	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		auto mace_dpg_trainer = std::static_pointer_cast<cMACEDPGTrainer>(mTrainers[i]);
		mace_dpg_trainer->SetActionFragSize(mActionFragSize);
	}
}

void cAsyncMACEDPGTrainer::SetTemp(double temp)
{
	mTemp = temp;
	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		auto mace_dpg_trainer = std::static_pointer_cast<cMACEDPGTrainer>(mTrainers[i]);
		mace_dpg_trainer->SetTemp(temp);
	}
}

void cAsyncMACEDPGTrainer::BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const
{
	out_trainer = std::shared_ptr<cNeuralNetTrainer>(new cMACEDPGTrainer());
}

void cAsyncMACEDPGTrainer::SetupTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer)
{
	cAsyncCaclaTrainer::SetupTrainer(out_trainer);

	auto mace_dpg_trainer = std::static_pointer_cast<cMACEDPGTrainer>(out_trainer);
	mace_dpg_trainer->SetNumActionFrags(mNumActionFrags);
	mace_dpg_trainer->SetActionFragSize(mActionFragSize);
	mace_dpg_trainer->SetTemp(mTemp);
}