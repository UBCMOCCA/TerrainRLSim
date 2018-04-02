#pragma once

#include "scenarios/ScenarioTrackMotion.h"

class cScenarioClimb : public cScenarioTrackMotion
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioClimb();
	virtual ~cScenarioClimb();
	
	virtual void Init();
	virtual void Update(double time_elapsed);
	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Reset();
	virtual void Clear();

	virtual tVector CalcLandingTarget() const;
	virtual double CalcLandingTargetErr() const;
	virtual double CalcCOMErr() const;
	virtual double CalcTargetVelErr() const;

	virtual std::string GetName() const;

protected:
	int mStepCount;
	tVector mLandingTargetStep;
	tVector mLandingTargetOffset;
	tVector mFrontLandingPos;

	virtual void ResetParams();

	virtual void UpdateKinChar(double time_step);
	virtual void SyncKinChar();
	virtual void CalcSyncKinPose(Eigen::VectorXd& out_pose) const;

	virtual void PostSubstepUpdate(double time_step);
	virtual void UpdateLandingPos();
};