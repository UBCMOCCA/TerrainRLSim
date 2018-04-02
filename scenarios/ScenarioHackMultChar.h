#pragma once

#include "scenarios/ScenarioPoliEval.h"

class cScenarioHackMultChar : virtual public cScenarioPoliEval
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioHackMultChar();
	virtual ~cScenarioHackMultChar();

	virtual void Init();
	virtual const std::shared_ptr<cSimCharacter>& GetHackCharacter() const;

	virtual std::string GetName() const;

protected:

	std::shared_ptr<cSimCharacter> mChar1;

	virtual void BuildChar1();
	virtual void UpdateCharacter(double time_step);
	virtual void Reset();
};