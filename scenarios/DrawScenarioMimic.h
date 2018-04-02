#pragma once

#include "DrawScenarioTrain.h"

class cKinCharacter;

class cDrawScenarioMimic: public cDrawScenarioTrain
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioMimic(cCamera& cam);
	virtual ~cDrawScenarioMimic();

protected:

	virtual void BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const;
	virtual void DrawCharacters() const;
	virtual void DrawKinChar() const;
	virtual const std::shared_ptr<cKinCharacter>& GetKinChar() const;
};