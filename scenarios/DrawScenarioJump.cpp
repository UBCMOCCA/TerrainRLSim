#include "DrawScenarioJump.h"
#include "ScenarioJump.h"

cDrawScenarioJump::cDrawScenarioJump(cCamera& cam)
	: cDrawScenarioTrackMotion(cam)
{
}

cDrawScenarioJump::~cDrawScenarioJump()
{
}

void cDrawScenarioJump::Update(double time_elapsed)
{
	cDrawScenarioTrackMotion::Update(time_elapsed);
	HandleJump();
}

void cDrawScenarioJump::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::unique_ptr<cScenarioSimChar>(new cScenarioJump());
}

void cDrawScenarioJump::HandleJump()
{
	cScenarioJump* jump_scene = reinterpret_cast<cScenarioJump*>(mScene.get());
	if (jump_scene->JumpCompleted())
	{
		tVector jump_dist = jump_scene->GetJumpDist();
		printf("Jump dist: %.5f\n", jump_dist[0]);
		jump_scene->ResetJumpCompleted();
	}
}