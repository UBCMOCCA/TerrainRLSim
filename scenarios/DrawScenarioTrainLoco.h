#pragma once

#include "DrawScenarioTrainCacla.h"

class cDrawScenarioTrainLoco : public cDrawScenarioTrainCacla
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioTrainLoco(cCamera& cam);
	virtual ~cDrawScenarioTrainLoco();

protected:

	virtual void BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const;
};