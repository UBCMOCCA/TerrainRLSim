#pragma once
#include <memory>

#include "DrawScenarioTrackMotion.h"

class cDrawScenarioJump : public cDrawScenarioTrackMotion
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioJump(cCamera& cam);
	virtual ~cDrawScenarioJump();

	virtual void Update(double time_elapsed);

protected:
	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;
	virtual void HandleJump();
};