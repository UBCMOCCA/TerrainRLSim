#pragma once

#include "scenarios/ScenarioExp.h"
#include "learning/MACETrainer.h"
#include "sim/DogControllerMACE.h"
#include "sim/GoatControllerMACE.h"
#include "sim/RaptorControllerMACE.h"
#include "sim/MonopedHopperControllerMACE.h"

class cScenarioExpMACE : virtual public cScenarioExp
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioExpMACE();
	virtual ~cScenarioExpMACE();

	virtual std::string GetName() const;

protected:
	
	virtual void RecordFlagsBeg(tExpTuple& out_tuple) const;
	
	virtual bool CheckExpCritic() const;
	virtual bool CheckExpActor() const;
};
