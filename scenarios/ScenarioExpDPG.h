#pragma once

#include "scenarios/ScenarioExpCacla.h"

class cScenarioExpDPG : virtual public cScenarioExpCacla
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioExpDPG();
	virtual ~cScenarioExpDPG();

	virtual std::string GetName() const;

protected:

	virtual bool EnableRandInitAction() const;
};