#pragma once

#include "scenarios/ScenarioExpDPG.h"

class cScenarioExpMACEDPG : virtual public cScenarioExpDPG
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioExpMACEDPG();
	virtual ~cScenarioExpMACEDPG();

	virtual std::string GetName() const;

protected:
};