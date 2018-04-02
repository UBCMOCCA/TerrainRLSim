#include "MixController.h"

cMixController::cMixController()
{
}

cMixController::~cMixController()
{
}

void cMixController::Init(cSimCharacter* character)
{
	cCharController::Init(character);
	mControllers.clear();
	mTaus.clear();
}

void cMixController::Reset()
{
	cCharController::Reset();
	for (int i = 0; i < GetNumControllers(); ++i)
	{
		mControllers[i]->Reset();
	}
}

void cMixController::Clear()
{
	cCharController::Clear();
	for (int i = 0; i < GetNumControllers(); ++i)
	{
		mControllers[i]->Clear();
	}
	mControllers.clear();
	mTaus.clear();
}

int cMixController::AddController(const std::shared_ptr<cTerrainRLCharController>& ctrl)
{
	int id = GetNumControllers();
	mControllers.push_back(ctrl);
	mTaus.resize(GetNumControllers());

	Eigen::VectorXd old_weights = mWeights;
	mWeights.resize(GetNumControllers());
	mWeights.segment(0, id) = old_weights;
	mWeights[id] = 1;

	return id;
}

void cMixController::Update(double time_step)
{
	cCharController::Update(time_step);

	Eigen::VectorXd tau;
	UpdateCalcTau(time_step, tau);
	UpdateApplyTau(tau);
}

void cMixController::UpdateCalcTau(double time_step, Eigen::VectorXd& out_tau)
{
	Eigen::VectorXd curr_tau;
	int num_ctrl = GetNumControllers();

	{
		int i = 0;
		const auto& ctrl = mControllers[i];
		ctrl->UpdateCalcTau(time_step, curr_tau);
		mTaus[i] = curr_tau;

		double w = mWeights[i];
		out_tau = w * curr_tau;
	}

	for (int i = 1; i < num_ctrl; ++i)
	{
		const auto& ctrl = mControllers[i];
		ctrl->UpdateCalcTau(time_step, curr_tau);
		assert(curr_tau.size() == out_tau.size());
		mTaus[i] = curr_tau;

		double w = mWeights[i];
		out_tau += w * curr_tau;
	}
}

void cMixController::UpdateApplyTau(const Eigen::VectorXd& tau)
{
	const auto& def_ctrl = GetDefaultController();
	def_ctrl->UpdateApplyTau(tau);
}

const Eigen::VectorXd& cMixController::GetTau() const
{
	const auto& def_ctrl = GetDefaultController();
	return def_ctrl->GetTau();
}

const Eigen::VectorXd& cMixController::GetTau(int id) const
{
	return mTaus[id];
}

int cMixController::GetNumControllers() const
{
	return static_cast<int>(mControllers.size());
}

const std::shared_ptr<cTerrainRLCharController>& cMixController::GetController(int id) const
{
	return mControllers[id];
}

const std::shared_ptr<cTerrainRLCharController>& cMixController::GetDefaultController() const
{
	return GetController(0);
}

void cMixController::SetWeight(int id, double w)
{
	mWeights[id] = w;
}

bool cMixController::NewActionUpdate() const
{
	const auto& ctrl = GetDefaultController();
	return ctrl->NewActionUpdate();
}

int cMixController::GetState() const
{
	const auto& ctrl = GetDefaultController();
	return ctrl->GetState();
}

double cMixController::GetPhase() const
{
	const auto& ctrl = GetDefaultController();
	return ctrl->GetPhase();
}

void cMixController::SetPhase(double phase)
{
	for (int i = 0; i < GetNumControllers(); ++i)
	{
		mControllers[i]->SetPhase(phase);
	}
}

int cMixController::GetNumStates() const
{
	const auto& ctrl = GetDefaultController();
	return ctrl->GetNumStates();
}

double cMixController::CalcNormPhase() const
{
	const auto& ctrl = GetDefaultController();
	return ctrl->CalcNormPhase();
}

void cMixController::TransitionState(int state)
{
	for (int i = 0; i < GetNumControllers(); ++i)
	{
		mControllers[i]->TransitionState(state);
	}
}

void cMixController::TransitionState(int state, double phase)
{
	for (int i = 0; i < GetNumControllers(); ++i)
	{
		mControllers[i]->TransitionState(state, phase);
	}
}

bool cMixController::IsNewCycle() const
{
	const auto& ctrl = GetDefaultController();
	return ctrl->IsNewCycle();
}

void cMixController::CommandAction(int action_id)
{
	for (int i = 0; i < GetNumControllers(); ++i)
	{
		mControllers[i]->CommandAction(action_id);
	}
}

void cMixController::CommandRandAction()
{
	for (int i = 0; i < GetNumControllers(); ++i)
	{
		mControllers[i]->CommandRandAction();
	}
}

int cMixController::GetDefaultAction() const
{
	const auto& ctrl = GetDefaultController();
	return ctrl->GetDefaultAction();
}

void cMixController::SetDefaultAction(int action_id)
{
	for (int i = 0; i < GetNumControllers(); ++i)
	{
		mControllers[i]->SetDefaultAction(action_id);
	}
}

int cMixController::GetNumActions() const
{
	const auto& ctrl = GetDefaultController();
	return ctrl->GetNumActions();
}

int cMixController::GetCurrActionID() const
{
	const auto& ctrl = GetDefaultController();
	return ctrl->GetCurrActionID();
}

void cMixController::EnableExp(bool enable)
{
	for (int i = 0; i < GetNumControllers(); ++i)
	{
		mControllers[i]->EnableExp(enable);
	}
}

const cMixController::tExpParams& cMixController::GetExpParams() const
{
	const auto& ctrl = GetDefaultController();
	return ctrl->GetExpParams();
}

void cMixController::SetExpParams(const tExpParams& params)
{
	for (int i = 0; i < GetNumControllers(); ++i)
	{
		mControllers[i]->SetExpParams(params);
	}
}

double cMixController::GetViewDist() const
{
	const auto& ctrl = GetDefaultController();
	return ctrl->GetViewDist();
}

void cMixController::SetViewDist(double dist)
{
	for (int i = 0; i < GetNumControllers(); ++i)
	{
		mControllers[i]->SetViewDist(dist);
	}
}

void cMixController::BuildNormPose(Eigen::VectorXd& pose) const
{
	const auto& ctrl = GetDefaultController();
	ctrl->BuildNormPose(pose);
}

void cMixController::BuildFromMotion(int ctrl_params_idx, const cMotion& motion)
{
	const auto& ctrl = GetDefaultController();
	ctrl->BuildFromMotion(ctrl_params_idx, motion);
}

void cMixController::BuildCtrlOptParams(int ctrl_params_idx, Eigen::VectorXd& out_params) const
{
	const auto& ctrl = GetDefaultController();
	ctrl->BuildCtrlOptParams(ctrl_params_idx, out_params);
}

void cMixController::SetCtrlOptParams(int ctrl_params_idx, const Eigen::VectorXd& params)
{
	for (int i = 0; i < GetNumControllers(); ++i)
	{
		mControllers[i]->SetCtrlOptParams(ctrl_params_idx, params);
	}
}

void cMixController::BuildActionOptParams(int action_id, Eigen::VectorXd& out_params) const
{
	const auto& ctrl = GetDefaultController();
	ctrl->BuildActionOptParams(action_id, out_params);
}

int cMixController::GetNumGroundSamples() const
{
	const auto& ctrl = GetDefaultController();
	return ctrl->GetNumGroundSamples();
}

tVector cMixController::GetGroundSample(int s) const
{
	const auto& ctrl = GetDefaultController();
	return ctrl->GetGroundSample(s);
}

tMatrix cMixController::GetGroundSampleTrans() const
{
	const auto& ctrl = GetDefaultController();
	return ctrl->GetGroundSampleTrans();
}

void cMixController::BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	const auto& ctrl = GetDefaultController();
	return ctrl->BuildNNOutputOffsetScale(out_offset, out_scale);
}