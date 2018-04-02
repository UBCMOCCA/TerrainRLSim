#pragma once
#include <memory>

#include "DrawScenarioPoliEval.h"

class cDrawScenarioHackMultChar : public cDrawScenarioPoliEval
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioHackMultChar(cCamera& cam);
	virtual ~cDrawScenarioHackMultChar();

	virtual void Keyboard(unsigned char key, int x, int y);

protected:
	
	bool mDrawOtherChar;

	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;
	virtual void DrawCharacters() const;
};