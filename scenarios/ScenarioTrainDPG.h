#pragma once

#include "scenarios/ScenarioTrainCacla.h"

class cScenarioTrainDPG : public cScenarioTrainCacla
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioTrainDPG();
	virtual ~cScenarioTrainDPG();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);

	virtual std::string GetName() const;

protected:
	double mQDiff;
	double mDPGReg;

	virtual void BuildTrainer(std::shared_ptr<cTrainerInterface>& out_trainer);
	virtual void BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const;
};