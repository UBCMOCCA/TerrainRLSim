#pragma once

#include "scenarios/ScenarioExpMimic.h"

class cScenarioExpMimicRNN : virtual public cScenarioExpMimic
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioExpMimicRNN();
	virtual ~cScenarioExpMimicRNN();

	virtual std::string GetName() const;

protected:

	virtual int GetNumWarmupCycles() const;
};