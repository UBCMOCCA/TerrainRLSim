#pragma once
#include <memory>

#include "DrawScenarioTrainCacla.h"

class cDrawScenarioTrainDPG: public cDrawScenarioTrainCacla
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioTrainDPG(cCamera& cam);
	virtual ~cDrawScenarioTrainDPG();

protected:

	virtual void BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const;
};