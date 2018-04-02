#pragma once

#include "learning/AsyncCaclaTrainer.h"

class cAsyncDMACETrainer : public cAsyncCaclaTrainer
{
public:

	cAsyncDMACETrainer();
	virtual ~cAsyncDMACETrainer();

	virtual void SetNumActionFrags(int num);
	virtual void SetActionFragSize(int size);
	virtual void SetTemp(double temp);
	virtual void SetGateScale(double scale);

protected:
	int mNumActionFrags;
	int mActionFragSize;
	double mTemp;
	double mGateScale;

	virtual void BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const;
	virtual void SetupTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer);
};