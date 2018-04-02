#pragma once
#include <memory>

#include "DrawScenarioTrackMotion.h"

class cDrawScenarioClimb : public cDrawScenarioTrackMotion
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioClimb(cCamera& cam);
	virtual ~cDrawScenarioClimb();

	virtual void Update(double time_elapsed);

protected:
	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;

	virtual void DrawScene();
	virtual void DrawLandingTarget();
};