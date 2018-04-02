#pragma once

#include "scenarios/ScenarioMimic.h"

class cScenarioMimicRNN : public cScenarioMimic
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioMimicRNN();
	virtual ~cScenarioMimicRNN();

	virtual std::string GetName() const;

protected:

	virtual void BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const;
	virtual void BuildTrainer(std::shared_ptr<cTrainerInterface>& out_trainer);
};