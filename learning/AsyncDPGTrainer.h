#pragma once

#include "learning/AsyncCaclaTrainer.h"

class cAsyncDPGTrainer : public cAsyncCaclaTrainer
{
public:

	cAsyncDPGTrainer();
	virtual ~cAsyncDPGTrainer();

	virtual void SetDPGReg(double reg);
	virtual void SetQDiff(double diff);
	virtual void SetTargetLerp(double lerp);
	
protected:
	double mDPGReg;
	double mQDiff;
	double mTargetLerp;

	virtual void BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const;
	virtual void SetupTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer);
	
};