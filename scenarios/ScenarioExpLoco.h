#pragma once

#include "scenarios/ScenarioExpCacla.h"

class cScenarioExpLoco : virtual public cScenarioExpCacla
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioExpLoco();
	virtual ~cScenarioExpLoco();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);

	virtual std::string GetName() const;

protected:
	
	tVector mTargetVel;

	virtual double CalcReward() const;
};