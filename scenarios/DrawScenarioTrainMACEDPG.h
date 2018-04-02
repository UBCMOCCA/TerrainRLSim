#pragma once
#include <memory>

#include "DrawScenarioTrainDPG.h"

class cDrawScenarioTrainMACEDPG : public cDrawScenarioTrainDPG
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioTrainMACEDPG(cCamera& cam);
	virtual ~cDrawScenarioTrainMACEDPG();

protected:

	virtual void BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const;
};