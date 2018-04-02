#pragma once
#include "scenarios/DrawScenarioSimChar.h"

class cDrawScenarioTerrainViewer : public cDrawScenarioSimChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioTerrainViewer(cCamera& cam);
	virtual ~cDrawScenarioTerrainViewer();

	virtual void Update(double time_elapsed);

protected:

	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;
	virtual void UpdateTargetCharPos();

	virtual tVector GetCamTrackPos() const;
	virtual tVector GetCamStillPos() const;
};