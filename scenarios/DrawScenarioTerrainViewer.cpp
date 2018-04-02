#include "scenarios/DrawScenarioTerrainViewer.h"
#include "scenarios/ScenarioTerrainViewer.h"

cDrawScenarioTerrainViewer::cDrawScenarioTerrainViewer(cCamera& cam)
							: cDrawScenarioSimChar(cam)
{
	tVector cam_pos = cam.GetPosition();
	tVector cam_focus = cam.GetFocus();
	double cam_dist = (cam_focus - cam_pos).norm();
	cam.SetPosition(cam_focus + tVector(0, cam_dist, 0, 0));
	cam.SetUp(tVector(0, 0, -1, 0));
}

cDrawScenarioTerrainViewer::~cDrawScenarioTerrainViewer()
{
}

void cDrawScenarioTerrainViewer::Update(double time_elapsed)
{
	cDrawScenarioSimChar::Update(time_elapsed);
	UpdateTargetCharPos();
}

void cDrawScenarioTerrainViewer::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioTerrainViewer>(new cScenarioTerrainViewer());
}

void cDrawScenarioTerrainViewer::UpdateTargetCharPos()
{
	tVector cam_focus = mCam.GetFocus();
	auto scene = std::dynamic_pointer_cast<cScenarioTerrainViewer>(mScene);
	scene->SetTargetCharPos(cam_focus);
}

tVector cDrawScenarioTerrainViewer::GetCamTrackPos() const
{
	return mCam.GetFocus();
}

tVector cDrawScenarioTerrainViewer::GetCamStillPos() const
{
	return mCam.GetFocus();
}