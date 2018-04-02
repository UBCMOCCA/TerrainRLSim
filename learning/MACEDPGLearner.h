#pragma once

#include "ACLearner.h"

struct cMACEDPGLearner : public cACLearner
{
public:
	cMACEDPGLearner(const std::shared_ptr<cNeuralNetTrainer>& trainer);
	virtual ~cMACEDPGLearner();

	virtual void SetTemp(double temp);
	virtual double GetTemp() const;

protected:
	double mTemp;

	virtual void UpdateTrainer();
};