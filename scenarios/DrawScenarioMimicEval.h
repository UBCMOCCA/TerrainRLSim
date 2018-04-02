#pragma once
#include "DrawScenarioPoliEval.h"

class cDrawScenarioMimicEval : public cDrawScenarioPoliEval
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioMimicEval(cCamera& cam);
	virtual ~cDrawScenarioMimicEval();

protected:
	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;
};