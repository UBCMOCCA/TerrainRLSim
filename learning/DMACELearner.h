#pragma once

#include "ACLearner.h"

struct cDMACELearner : public cACLearner
{
public:
	cDMACELearner(const std::shared_ptr<cNeuralNetTrainer>& trainer);
	virtual ~cDMACELearner();

	virtual void SetTemp(double temp);
	virtual double GetTemp() const;

protected:
	double mTemp;

	virtual void UpdateTrainer();
};