#pragma once

#include "scenarios/ScenarioExpCacla.h"
#include "learning/DMACETrainer.h"
#include "sim/BaseControllerMACE.h"

class cScenarioExpDMACE : virtual public cScenarioExpCacla
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioExpDMACE();
	virtual ~cScenarioExpDMACE();

	virtual std::string GetName() const;

protected:
	
	virtual void RecordFlagsBeg(tExpTuple& out_tuple) const;
	virtual bool CheckExpCritic() const;
	virtual bool CheckExpActor() const;
};