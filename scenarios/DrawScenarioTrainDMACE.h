#pragma once
#include <memory>

#include "DrawScenarioTrainCacla.h"

class cDrawScenarioTrainDMACE: public cDrawScenarioTrainCacla
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioTrainDMACE(cCamera& cam);
	virtual ~cDrawScenarioTrainDMACE();

protected:

	virtual void BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const;
};