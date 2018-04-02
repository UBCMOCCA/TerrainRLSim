#pragma once

#include "learning/AsyncDPGTrainer.h"

class cAsyncMACEDPGTrainer : public cAsyncDPGTrainer
{
public:

	cAsyncMACEDPGTrainer();
	virtual ~cAsyncMACEDPGTrainer();

	virtual void SetNumActionFrags(int num);
	virtual void SetActionFragSize(int size);
	virtual void SetTemp(double temp);

protected:
	int mNumActionFrags;
	int mActionFragSize;
	double mTemp;
	
	virtual void BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const;
	virtual void SetupTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer);
};