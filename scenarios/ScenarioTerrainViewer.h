#pragma once

#include "scenarios/ScenarioSimChar.h"

class cScenarioTerrainViewer : public cScenarioSimChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioTerrainViewer();
	virtual ~cScenarioTerrainViewer();

	virtual void Reset();
	virtual void SetTargetCharPos(const tVector& pos);

	virtual std::string GetName() const;

protected:

	tVector mTargetCharPos;

	virtual void UpdateCharacter(double time_step);
	virtual void MoveCharacter(const tVector& pos);

	virtual void GetViewBound(tVector& out_min, tVector& out_max) const;
};
