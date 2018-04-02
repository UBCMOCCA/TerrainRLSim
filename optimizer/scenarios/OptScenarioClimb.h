#pragma once

#include "scenarios/OptScenarioTrackMotion.h"

class cOptScenarioClimb : public  cOptScenarioTrackMotion
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cOptScenarioClimb(cOptimizer& optimizer);
	virtual ~cOptScenarioClimb();

	virtual std::string GetName() const;

protected:
	virtual void BuildScene(int id, std::unique_ptr<cScenarioSimChar>& out_scene);
	virtual double CalcCurrErr(const std::unique_ptr<cScenarioSimChar>& scene);
};