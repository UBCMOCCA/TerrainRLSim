#pragma once

#include <string>

#include "Scenario.h"
#include "anim/KinCharacter.h"

class cScenarioKinMotion : public cScenario
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioKinMotion();
	virtual ~cScenarioKinMotion();

	virtual void Init();
	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Reset();
	virtual void Clear();

	virtual void Update(double time_elapsed);

	virtual const cKinCharacter& GetCharacter() const;
	virtual tVector GetCharPos() const;
	virtual double GetTime() const;

	virtual std::string GetName() const;

protected:
	std::string mCharFile;
	std::string mMotionFile;

	tVector mCharInitPos;

	cKinCharacter mChar;
	
	virtual bool BuildCharacter(const std::string& char_file, const std::string& motion_file);
};