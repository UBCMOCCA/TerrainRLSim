#include "DrawScenarioMimic.h"
#include "scenarios/ScenarioMimic.h"
#include "scenarios/ScenarioExpMimic.h"
#include "anim/KinCharacter.h"
#include "render/DrawUtil.h"
#include "render/DrawCharacter.h"

const double gLinkWidth = 0.025;
const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gFilLColor = tVector(0.6, 0.65, 0.675, 1);
const tVector gKinCharOffset = tVector(0, 0, 0.5, 0);

cDrawScenarioMimic::cDrawScenarioMimic(cCamera& cam)
	: cDrawScenarioTrain(cam)
{
}

cDrawScenarioMimic::~cDrawScenarioMimic()
{
}

void cDrawScenarioMimic::BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioTrain>(new cScenarioMimic());
}


void cDrawScenarioMimic::DrawCharacters() const
{
	cDrawScenarioTrain::DrawCharacters();
	DrawKinChar();
}

void cDrawScenarioMimic::DrawKinChar() const
{
	const auto& kin_char = GetKinChar();
	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(gKinCharOffset);
	cDrawCharacter::Draw(*kin_char.get(), gLinkWidth, gFilLColor, gLineColor);
	cDrawUtil::PopMatrix();
}

const std::shared_ptr<cKinCharacter>& cDrawScenarioMimic::GetKinChar() const
{
	auto scene = std::dynamic_pointer_cast<cScenarioExpMimic>(mScene);
	return scene->GetKinChar();
}