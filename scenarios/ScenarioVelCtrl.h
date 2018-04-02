#pragma once

#include "scenarios/ScenarioTrackMotion.h"

class cScenarioVelCtrl : public cScenarioTrackMotion
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioVelCtrl();
	virtual ~cScenarioVelCtrl();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Init();
	virtual void Reset();
	virtual void Clear();
	virtual void Update(double time_elapsed);

	virtual std::string GetName() const;

protected:
	double mCommandMaxTime;
	double mCommandTimer;
	int mCurrTargetIdx;
	std::vector<double> mTargetVels;

	virtual void ResetParams();
	virtual bool BuildController(std::shared_ptr<cCharController>& out_ctrl);

	virtual void UpdateCommand(double time_step);
	virtual double CalcTargetVelErr() const;
};