#pragma once
#include "scenarios/DrawScenarioPoliEval.h"

class cDrawScenarioLocoEval : public cDrawScenarioPoliEval
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioLocoEval(cCamera& cam);
	virtual ~cDrawScenarioLocoEval();

protected:

	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;
};