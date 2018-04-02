#pragma once

#include "scenarios/ScenarioTrainCacla.h"

class cScenarioTrainLoco : public cScenarioTrainCacla
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioTrainLoco();
	virtual ~cScenarioTrainLoco();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);

	virtual std::string GetName() const;

protected:
	
	virtual void BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const;
};