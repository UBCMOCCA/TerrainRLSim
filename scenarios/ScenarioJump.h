#pragma once

#include "scenarios/ScenarioTrackMotion.h"

class cScenarioJump : public cScenarioTrackMotion
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioJump();
	virtual ~cScenarioJump();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Init();
	virtual void Reset();
	virtual void Clear();
	virtual void Update(double time_elapsed);

	virtual bool JumpCompleted() const;
	virtual bool Jumping() const;
	virtual void ResetJumpCompleted();
	virtual tVector GetJumpDist() const;

	virtual double CalcPoseThetaRelErr() const;
	virtual double CalcJumpHeightReward() const;

	virtual std::string GetName() const;

protected:
	enum eJumpState
	{
		eJumpStateRecovery,
		eJumpStateCommand,
		eJumpStateJump,
		eJumpStateMax
	};

	eJumpState mJumpState;
	tVector mJumpStartPos;
	tVector mJumpEndPos;

	double mWarmUpTime;
	double mRecoveryTime;
	double mRecoveryTimer;
	int mCurrTargetIdx;

	bool mJumpCompleted; // just for training to indicate a new jump has been completed

	virtual void ResetParams();
	virtual bool BuildController(std::shared_ptr<cCharController>& out_ctrl);

	virtual void PreSubstepUpdate(double time_step);
	virtual void UpdateJump(double time_step);
	virtual bool IsNewCycle() const;

	virtual void UpdateKinChar(double time_step);
	virtual void SyncKinChar();
};