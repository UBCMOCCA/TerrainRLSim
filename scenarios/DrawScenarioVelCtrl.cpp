#include "DrawScenarioVelCtrl.h"
#include "ScenarioVelCtrl.h"

cDrawScenarioVelCtrl::cDrawScenarioVelCtrl(cCamera& cam)
	: cDrawScenarioTrackMotion(cam)
{
}

cDrawScenarioVelCtrl::~cDrawScenarioVelCtrl()
{
}

void cDrawScenarioVelCtrl::Update(double time_elapsed)
{
	cDrawScenarioTrackMotion::Update(time_elapsed);
}

void cDrawScenarioVelCtrl::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::unique_ptr<cScenarioSimChar>(new cScenarioVelCtrl());
}