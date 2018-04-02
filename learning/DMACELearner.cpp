#include "DMACELearner.h"
#include "DMACETrainer.h"
#include "ACTrainer.h"

cDMACELearner::cDMACELearner(const std::shared_ptr<cNeuralNetTrainer>& trainer)
	: cACLearner(trainer)
{
	mTemp = 1;
}

cDMACELearner::~cDMACELearner()
{
}

void cDMACELearner::SetTemp(double temp)
{
	mTemp = temp;
}

double cDMACELearner::GetTemp() const
{
	return mTemp;
}

void cDMACELearner::UpdateTrainer()
{
	auto trainer = std::static_pointer_cast<cDMACETrainer>(mTrainer);
	trainer->SetTemp(GetTemp());
}