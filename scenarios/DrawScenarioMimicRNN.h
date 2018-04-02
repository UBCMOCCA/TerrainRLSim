#pragma once

#include "DrawScenarioMimic.h"

class cDrawScenarioMimicRNN: public cDrawScenarioMimic
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioMimicRNN(cCamera& cam);
	virtual ~cDrawScenarioMimicRNN();

protected:

	virtual void BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const;
};