#pragma once

#include "OptScenarioPoliEval.h"

class cOptScenarioLocoEval : public cOptScenarioPoliEval
{
public:
	cOptScenarioLocoEval();
	virtual ~cOptScenarioLocoEval();

protected:
	
	virtual void BuildScene(int id, std::shared_ptr<cScenarioPoliEval>& out_scene) const;
};