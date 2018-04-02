#pragma once
#include <memory>

#include "DrawScenarioTrackMotion.h"

class cDrawScenarioVelCtrl : public cDrawScenarioTrackMotion
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioVelCtrl(cCamera& cam);
	virtual ~cDrawScenarioVelCtrl();

	virtual void Update(double time_elapsed);

protected:
	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;
};