#include "ScenarioMimicEval.h"

cScenarioMimicEval::cScenarioMimicEval() :
	cScenarioExpMimic(),
	cScenarioPoliEval()
{
	mTauErr = 0;
	mTauErrCount = 0;
}

cScenarioMimicEval::~cScenarioMimicEval()
{
}

void cScenarioMimicEval::GetTauErrResult(double& out_err, int& out_count) const
{
	out_err = mTauErr;
	out_count = mTauErrCount;
}

std::string cScenarioMimicEval::GetName() const
{
	return "Mimic Evaluation";
}

void cScenarioMimicEval::ResetRecord()
{
	cScenarioPoliEval::ResetRecord();
	mTauErr = 0;
	mTauErrCount = 0;
}

void cScenarioMimicEval::PostSubstepUpdateMisc(double time_step)
{
	cScenarioPoliEval::PostSubstepUpdateMisc(time_step);
	UpdateTauErr();
}

void cScenarioMimicEval::UpdateTauErr()
{
	const auto& ctrl = std::static_pointer_cast<cTerrainRLCharController>(mChar->GetController());
	const Eigen::VectorXd& ctrl_tau = ctrl->GetTau();

	Eigen::VectorXd diff = ctrl_tau - mCoachAction;
	double tau_err = diff.norm();

	mTauErr = cMathUtil::AddAverage(mTauErr, mTauErrCount, tau_err, 1);
	++mTauErrCount;

#if defined (ENABLE_DEBUG_PRINT)
	if (mTauErrCount % 100 == 0)
	{
		printf("Avg tau err: %.5f (%i)\n", mTauErr, mTauErrCount);
	}
#endif
}

bool cScenarioMimicEval::EnableRandInitAction() const
{
	return cScenarioPoliEval::EnableRandInitAction();
}

void cScenarioMimicEval::InitMisc()
{
	cScenarioPoliEval::InitMisc();
	mTauErr = 0;
	mTauErrCount = 0;
}