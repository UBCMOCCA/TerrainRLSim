#include "MACEDPGLearner.h"
#include "MACEDPGTrainer.h"

cMACEDPGLearner::cMACEDPGLearner(const std::shared_ptr<cNeuralNetTrainer>& trainer)
	: cACLearner(trainer)
{
	mTemp = 1;
}

cMACEDPGLearner::~cMACEDPGLearner()
{
}

void cMACEDPGLearner::SetTemp(double temp)
{
	mTemp = temp;
}

double cMACEDPGLearner::GetTemp() const
{
	return mTemp;
}

void cMACEDPGLearner::UpdateTrainer()
{
	auto trainer = std::static_pointer_cast<cMACEDPGTrainer>(mTrainer);
	trainer->SetTemp(GetTemp());
}