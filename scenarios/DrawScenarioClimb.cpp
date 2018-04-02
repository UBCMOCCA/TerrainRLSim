#include "DrawScenarioClimb.h"
#include "ScenarioClimb.h"
#include "render/DrawUtil.h"

cDrawScenarioClimb::cDrawScenarioClimb(cCamera& cam)
	: cDrawScenarioTrackMotion(cam)
{
}

cDrawScenarioClimb::~cDrawScenarioClimb()
{
}

void cDrawScenarioClimb::Update(double time_elapsed)
{
	cDrawScenarioTrackMotion::Update(time_elapsed);
}

void cDrawScenarioClimb::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::unique_ptr<cScenarioSimChar>(new cScenarioClimb());
}

void cDrawScenarioClimb::DrawScene()
{
	cDrawScenarioTrackMotion::DrawScene();
	DrawLandingTarget();
}

void cDrawScenarioClimb::DrawLandingTarget()
{
	std::shared_ptr<cScenarioClimb> scene = std::static_pointer_cast<cScenarioClimb>(mScene);
	tVector tar_pos = scene->CalcLandingTarget();

	cDrawUtil::SetLineWidth(3);
	cDrawUtil::SetColor(tVector(0, 0, 1, 0.5));
	cDrawUtil::DrawCross(tar_pos, 0.15);
}