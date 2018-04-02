#pragma once

#include "scenarios/ScenarioPoliEval.h"
#include "scenarios/ScenarioExpMimic.h"

class cScenarioMimicEval : virtual public cScenarioPoliEval, virtual public cScenarioExpMimic
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioMimicEval();
	virtual ~cScenarioMimicEval();

	virtual void ResetRecord();
	virtual void GetTauErrResult(double& out_err, int& out_count) const;

	virtual std::string GetName() const;

protected:

	double mTauErr;
	int mTauErrCount;

	virtual void PostSubstepUpdateMisc(double time_step);
	virtual void UpdateTauErr();

	virtual bool EnableRandInitAction() const;
	virtual void InitMisc();
};