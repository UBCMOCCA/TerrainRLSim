#include "DrawScenarioHackMultChar.h"
#include "scenarios/ScenarioHackMultChar.h"
#include "render/DrawSimCharacter.h"
#include "render/DrawUtil.h"

const tVector gFillTint = tVector(1, 1, 1, 1);

cDrawScenarioHackMultChar::cDrawScenarioHackMultChar(cCamera& cam)
	: cDrawScenarioPoliEval(cam)
{
	mDrawOtherChar = true;
}

cDrawScenarioHackMultChar::~cDrawScenarioHackMultChar()
{
}

void cDrawScenarioHackMultChar::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenarioPoliEval::Keyboard(key, x, y);

	switch (key)
	{
	case 'u':
		mDrawOtherChar = !mDrawOtherChar;
		break;
	default:
		break;
	}
}

void cDrawScenarioHackMultChar::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioHackMultChar>(new cScenarioHackMultChar());
}

void cDrawScenarioHackMultChar::DrawCharacters() const
{
	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(tVector(0, 0, -0.5, 0));
	cDrawScenarioPoliEval::DrawCharacters();
	cDrawUtil::PopMatrix();

	if (mDrawOtherChar)
	{
		cDrawUtil::PushMatrix();
		cDrawUtil::Translate(tVector(0, 0, 0.5, 0));
		auto scene = std::dynamic_pointer_cast<cScenarioHackMultChar>(mScene);
		const auto& character = scene->GetHackCharacter();
		bool enable_draw_shape = mDraw3D;
		cDrawSimCharacter::Draw(*(character.get()), gFillTint, GetLineColor(), enable_draw_shape);
		cDrawUtil::PopMatrix();
	}
}