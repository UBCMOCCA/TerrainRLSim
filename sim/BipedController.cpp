/*
 * BipedController.cpp
 *
 *  Created on: 2016-06-28
 *      Author: gberseth
 */

#include "BipedController.h"
#include <iostream>
#include <ctime>
#include "util/json/json.h"

#include "sim/SimCharacter.h"
#include "sim/RBDUtil.h"
#include "util/FileUtil.h"


const cBipedController::eStance gDefaultStance = cBipedController::eStanceRight;
const cBipedController::tStateDef gStateDefs[cBipedController::eStateMax] =
{
	{
		"Contact",
		true,									// mTransTime;
		false,									// mTransContact;
		cBipedController::eStateDown			// mNext;
	},
	{
		"Down",
		true,									// mTransTime;
		false,									// mTransFootContact;
		cBipedController::eStatePassing		// mNext;
	},
	{
		"Passing",
		true,									// mTransTime;
		false,									// mTransContact;
		cBipedController::eStateUp				// mNext;
	},
	{
		"Up",
		false,									// mTransTime;
		true,									// mTransContact;
		cBipedController::eStateInvalid		// mNext;
	},
};

const std::string gMiscParamsNames[cBipedController::eMiscParamMax] = {
	"TransTime",
	"Cv",
	"Cd",
	"ForceX",
	"ForceY"
};

const std::string gStateParamNames[cBipedController::eStateParamMax] = {
	"RootPitch",
	"SpineCurve",
	"StanceHip",
	"StanceKnee",
	"SwingHip",
	"SwingKnee"
};

const cSimBiped::eJoint gEndEffectors[] = {
	cSimBiped::eJointRightAnkle,
	cSimBiped::eJointLeftAnkle,
};
const int gNumEndEffectors = sizeof(gEndEffectors) / sizeof(gEndEffectors[0]);

const cSimBiped::eJoint gSpineJoints[] = {
	cSimBiped::eJointSpine0,
	cSimBiped::eJointSpine1,
	cSimBiped::eJointSpine2,
	cSimBiped::eJointSpine3
};
const int gNumSpineJoints = sizeof(gSpineJoints) / sizeof(gSpineJoints[0]);

const std::string gMiscParamsKey = "MiscParams";
const std::string gStateParamsKey = "StateParams";
const std::string gControllersKey = "Controllers";
const std::string gFilesKey = "Files";
const std::string gActionsKey = "Actions";

const bool gOptParamsMasks[] =
{
	false,	//TransTime
	true,	//Cv
	true,	//Cd
	false,	//ForceX
	false,	//ForceY

	//Contact
	true,	//RootPitch
	false,	//SpineCurve
	true,	//StanceHip
	true,	//StanceKnee
	true,	//StanceAnkle
	true,	//SwingHip
	true,	//SwingKnee
	true,	//SwingAnkle

	//Down
	true,	//RootPitch
	false,	//SpineCurve
	true,	//StanceHip
	true,	//StanceKnee
	true,	//StanceAnkle
	true,	//SwingHip
	true,	//SwingKnee
	true,	//SwingAnkle

	//Passing
	false,	//RootPitch
	false,	//SpineCurve
	true,	//StanceHip
	true,	//StanceKnee
	true,	//StanceAnkle
	true,	//SwingHip
	true,	//SwingKnee
	true,	//SwingAnkle

	//Up
	false,	//RootPitch
	false,	//SpineCurve
	true,	//StanceHip
	true,	//StanceKnee
	true,	//StanceAnkle
	true,	//SwingHip
	true,	//SwingKnee
	true	//SwingAnkle
};
const int gNumOptParamMasks = sizeof(gOptParamsMasks) / sizeof(gOptParamsMasks[0]);
const int gPosDim = cKinTree::gPosDim - 1;

cBipedController::cBipedController() : cTerrainRLCharController()
{
	mDefaultAction = gInvalidIdx;
	mState = eStateContact;
	mEnableGravityCompensation = true;
	mEnableVirtualForces = true;
}

cBipedController::~cBipedController()
{
}

void cBipedController::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	// param_file should contain parameters for the pd controllers
	cTerrainRLCharController::Init(character);

	mGravity = gravity;
	mCtrlParams.clear();
	mPrevCOM = character->CalcCOM();

	Eigen::MatrixXd pd_params;
	bool succ = cPDController::LoadParams(param_file, pd_params);

	if (succ)
	{
		mRBDModel = std::shared_ptr<cRBDModel>(new cRBDModel());
		mRBDModel->Init(mChar->GetJointMat(), mChar->GetBodyDefs(), mGravity);

		mImpPDCtrl.Init(mChar, mRBDModel, pd_params, mGravity);

		succ = LoadControllers(param_file);
		TransitionState(eStateContact);
	}

	if (succ)
	{
		mValid = true;
		SetStance(gDefaultStance);
	}
	else
	{
		printf("Failed to initialize dog controller\n");
		mValid = false;
	}
}

void cBipedController::Reset()
{
	cTerrainRLCharController::Reset();
	ClearCommands();
	mImpPDCtrl.Reset();
	SetStance(gDefaultStance);
	mPrevCOM = mChar->CalcCOM();
}

void cBipedController::Clear()
{
	cTerrainRLCharController::Clear();
	mImpPDCtrl.Clear();
	mCtrlParams.clear();
	ClearCommands();

	mRBDModel.reset();
	mDefaultAction = gInvalidIdx;
}

void cBipedController::Update(double time_step)
{
	cTerrainRLCharController::Update(time_step);
}

void cBipedController::UpdateCalcTau(double time_step, Eigen::VectorXd& out_tau)
{
	int num_dof = mChar->GetNumDof();
	out_tau = Eigen::VectorXd::Zero(num_dof);

	if (mMode == eModeActive)
	{
		mCurrCycleTime += time_step;
		UpdateStumbleCounter(time_step);

		UpdateRBDModel();
		UpdateState(time_step);
		UpdateStanceHip();

		// order matters!
		ApplySwingFeedback(out_tau);
		UpdatePDCtrls(time_step, out_tau);

		if (mEnableGravityCompensation)
		{
			ApplyGravityCompensation(out_tau);
		}

		ApplyStanceFeedback(out_tau);

		if (mEnableVirtualForces)
		{
			ApplyVirtualForces(out_tau);
		}
	}
	else if (mMode == eModePassive)
	{
		UpdatePDCtrls(time_step, out_tau);
	}
}

void cBipedController::UpdateApplyTau(const Eigen::VectorXd& tau)
{
	mTau = tau;
	mChar->ApplyControlForces(tau);
}

void cBipedController::SeCtrlStateParams(eState state, const tStateParams& params)
{
	mCurrAction.mParams.segment(eMiscParamMax + state * eStateParamMax, eStateParamMax) = params;
}

void cBipedController::TransitionState(int state)
{
	cTerrainRLCharController::TransitionState(state);
}

void cBipedController::TransitionState(int state, double phase)
{
	assert(state >= 0 && state < eStateMax);
	cTerrainRLCharController::TransitionState(state, phase);

	tStateParams params = GetCurrParams();
	SetStateParams(params);
}

void cBipedController::SetTransTime(double time)
{
	// set duration of each state in the walk
	mCurrAction.mParams[eMiscParamTransTime] = time;
}

int cBipedController::GetNumStates() const
{
	return eStateMax;
}

void cBipedController::SetMode(eMode mode)
{
	cTerrainRLCharController::SetMode(mode);

	if (mMode == eModePassive)
	{
		SetupPassiveMode();
	}
}

void cBipedController::CommandAction(int action_id)
{
	int num_actions = GetNumActions();
	if (action_id < num_actions)
	{
		mCommands.push(action_id);
	}
	else
	{
		assert(false); // invalid action
	}
}

void cBipedController::CommandRandAction()
{
	int num_actions = GetNumActions();
	int a = cMathUtil::RandInt(0, num_actions);
	CommandAction(a);
}

int cBipedController::GetDefaultAction() const
{
	return mDefaultAction;
}

void cBipedController::SetDefaultAction(int action_id)
{
	if (action_id >= 0 && action_id < GetNumActions())
	{
		mDefaultAction = action_id;
	}
}

int cBipedController::GetNumActions() const
{
	return static_cast<int>(mActions.size());
}

void cBipedController::BuildCtrlOptParams(int ctrl_params_idx, Eigen::VectorXd& out_params) const
{
	GetOptParams(mCtrlParams[ctrl_params_idx], out_params);
}

void cBipedController::SetCtrlParams(int ctrl_params_id, const Eigen::VectorXd& params)
{
	assert(params.size() == GetNumParams());
	Eigen::VectorXd& ctrl_params = mCtrlParams[ctrl_params_id];
	ctrl_params = params;
	PostProcessParams(ctrl_params);

	const tBlendAction& curr_action = mActions[mCurrAction.mID];
	if (curr_action.mParamIdx0 == ctrl_params_id
		|| curr_action.mParamIdx1 == ctrl_params_id)
	{
		ApplyAction(mCurrAction.mID);
	}
}

void cBipedController::SetCtrlOptParams(int ctrl_params_id, const Eigen::VectorXd& opt_params)
{
	assert(opt_params.size() == GetNumOptParams());
	Eigen::VectorXd& ctrl_params = mCtrlParams[ctrl_params_id];
	SetOptParams(opt_params, ctrl_params);

	const tBlendAction& curr_action = mActions[mCurrAction.mID];
	if (curr_action.mParamIdx0 == ctrl_params_id
		|| curr_action.mParamIdx1 == ctrl_params_id)
	{
		ApplyAction(mCurrAction.mID);
	}
}

void cBipedController::BuildActionOptParams(int action_id, Eigen::VectorXd& out_params) const
{
	assert(action_id >= 0 && action_id < GetNumActions());
	const tBlendAction& action = mActions[action_id];
	Eigen::VectorXd params;
	BlendCtrlParams(action, params);
	GetOptParams(params, out_params);
}

int cBipedController::GetNumParams() const
{
	int num_params = eMiscParamMax + eStateMax * eStateParamMax;
	return num_params;
}

int cBipedController::GetNumOptParams() const
{
	int num_params = GetNumParams();
	assert(gNumOptParamMasks == num_params);

	int num_opt_params = 0;
	for (int i = 0; i < num_params; ++i)
	{
		if (IsOptParam(i))
		{
			++num_opt_params;
		}
	}

	return num_opt_params;
}

void cBipedController::FetchOptParamScale(Eigen::VectorXd& out_scale) const
{
	int num_opt_params = GetNumOptParams();
	int num_params = GetNumParams();

	const double angle_scale = M_PI / 10;
	const double cv_scale = 0.02;
	const double cd_scale = 0.2;
	const double force_scale = 100;
	const double spine_curve_scale = 0.02;

	Eigen::VectorXd param_buffer = Eigen::VectorXd::Ones(num_params);

	int idx = 0;
	for (int i = 0; i < eMiscParamMax; ++i)
	{
		if (i == eMiscParamCv)
		{
			param_buffer[idx] = cv_scale;
		}
		else if (i == eMiscParamCd)
		{
			param_buffer[idx] = cd_scale;
		}
		else if (i == eMiscParamForceX || i == eMiscParamForceY)
		{
			param_buffer[idx] = force_scale;
		}

		++idx;
	}

	for (int i = 0; i < eStateMax; ++i)
	{
		for (int j = 0; j < eStateParamMax; ++j)
		{
			if (j == eStateParamSpineCurve)
			{
				param_buffer[idx] = spine_curve_scale;
			}
			else
			{
				param_buffer[idx] = angle_scale;
			}
			++idx;
		}
	}

	assert(idx == num_params);
	GetOptParams(param_buffer, out_scale);
}

void cBipedController::OutputOptParams(const std::string& file, const Eigen::VectorXd& params) const
{
	cController::OutputOptParams(file, params);
}

void cBipedController::OutputOptParams(FILE* f, const Eigen::VectorXd& params) const
{
	std::string opt_param_json = BuildOptParamsJson(params);
	fprintf(f, "%s\n", opt_param_json.c_str());
}

void cBipedController::ReadParams(std::ifstream& f_stream)
{
	Json::Reader reader;
	Json::Value root;
	bool succ = reader.parse(f_stream, root);
	if (succ)
	{
		double trans_time = 0;
		Eigen::VectorXd param_vec = Eigen::VectorXd::Zero(GetNumParams());
		int idx = 0;
		// grab the transition time first
		if (!root[gMiscParamsKey].isNull())
		{
			Json::Value misc_params = root[gMiscParamsKey];
			for (int i = 0; i < eMiscParamMax; ++i)
			{
				param_vec[idx++] = misc_params[gMiscParamsNames[i]].asDouble();
			}
		}
		else
		{
			printf("Failed to find %s\n", gMiscParamsKey.c_str());
		}

		if (!root[gStateParamsKey].isNull())
		{
			Json::Value state_params = root[gStateParamsKey];

			for (int i = 0; i < eStateMax; ++i)
			{
				Json::Value curr_params = state_params[gStateDefs[i].mName];
				for (int j = 0; j < eStateParamMax; ++j)
				{
					param_vec[idx++] = curr_params[gStateParamNames[j]].asDouble();
				}
			}
		}
		else
		{
			printf("Failed to find %s\n", gStateParamsKey.c_str());
		}

		if (!HasCtrlParams())
		{
			SetParams(param_vec);

			// refresh controls params
			tStateParams curr_params = GetCurrParams();
			SetStateParams(curr_params);
		}
		PostProcessParams(param_vec);
		mCtrlParams.push_back(param_vec);
	}
}

void cBipedController::ReadParams(const std::string& file)
{
	cTerrainRLCharController::ReadParams(file);
}

void cBipedController::SetParams(const Eigen::VectorXd& params)
{
	assert(params.size() == GetNumParams());
	mCurrAction.mParams = params;
	PostProcessParams(mCurrAction.mParams);
}

void cBipedController::BuildOptParams(Eigen::VectorXd& out_params) const
{
	GetOptParams(mCurrAction.mParams, out_params);
}

void cBipedController::SetOptParams(const Eigen::VectorXd& opt_params)
{
	SetOptParams(opt_params, mCurrAction.mParams);
}

void cBipedController::SetOptParams(const Eigen::VectorXd& opt_params, Eigen::VectorXd& out_params) const
{
	assert(opt_params.size() == GetNumOptParams());
	assert(gNumOptParamMasks == GetNumParams());

	int num_params = GetNumParams();
	int opt_idx = 0;
	for (int i = 0; i < num_params; ++i)
	{
		if (IsOptParam(i))
		{
			out_params[i] = opt_params[opt_idx];
			++opt_idx;
		}
	}

	PostProcessParams(out_params);
}

void cBipedController::BuildFromMotion(int ctrl_params_idx, const cMotion& motion)
{
	assert(motion.IsValid());
	double dur = motion.GetDuration();
	dur -= 0.00001; // hack to prevent flipping stance at the beginning of a new cycle
	int num_states = GetNumStates();
	double trans_time = dur / num_states;
	SetTransTime(trans_time);

	Eigen::VectorXd params = mCurrAction.mParams;
	for (int s = 0; s < num_states; ++s)
	{
		double curr_time = (s + 1) * trans_time;
		Eigen::VectorXd curr_pose = motion.CalcFrame(curr_time);

		tStateParams state_params = tStateParams::Zero();
		BuildStateParamsFromPose(curr_pose, state_params);
		params.segment(eMiscParamMax + s * eStateParamMax, eStateParamMax) = state_params;
	}

	SetCtrlParams(ctrl_params_idx, params);
}

double cBipedController::CalcReward() const
{
	tVector target_vel = GetTargetVel();

	const double vel_reward_w = 0.8;
	double vel_reward = 0;
	const double stumble_reward_w = 0.2;
	double stumble_reward = 0;

	bool fallen = mChar->HasFallen();
	if (!fallen)
	{
		double cycle_time = GetPrevCycleTime();
		double avg_vel = mPrevDistTraveled[0] / cycle_time;
		double vel_err = target_vel[0] - avg_vel;
		double vel_gamma = 0.5;
		vel_reward = std::exp(-vel_gamma * vel_err * vel_err);

		double stumble_count = mPrevStumbleCount;
		double avg_stumble = stumble_count /= cycle_time;
		double stumble_gamma = 10;
		stumble_reward = 1.0 / (1 + stumble_gamma * avg_stumble);

		if (avg_vel < 0)
		{
			vel_reward = 0;
			stumble_reward = 0;
		}
	}

	double reward = 0;
	reward += vel_reward_w * vel_reward
			+ stumble_reward_w * stumble_reward;

	return reward;
}

tVector cBipedController::GetTargetVel() const
{
	return tVector(4, 0, 0, 0);
	//return tVector(1.75, 0, 0, 0);
}

cBipedController::eStance cBipedController::GetStance() const
{
	return mStance;
}

double cBipedController::GetPrevCycleTime() const
{
	return mPrevCycleTime;
}

const tVector& cBipedController::GetPrevDistTraveled() const
{
	return mPrevDistTraveled;
}

void cBipedController::BuildNormPose(Eigen::VectorXd& pose) const
{
	if (mChar != nullptr)
	{
		pose = mChar->GetPose();
		eStance stance = GetStance();
		if (stance != gDefaultStance)
		{
			FlipPoseStance(pose);
		}
	}
}

bool cBipedController::LoadControllers(const std::string& file)
{
	std::ifstream f_stream(file);
	Json::Reader reader;
	Json::Value root;
	bool succ = reader.parse(f_stream, root);
	if (succ)
	{
		if (!root[gControllersKey].isNull())
		{
			Json::Value ctrl_root = root[gControllersKey];
			succ = ParseControllers(ctrl_root);
		}
	}

	if (succ)
	{
		if (mDefaultAction != gInvalidIdx)
		{
			ApplyAction(mDefaultAction);
		}
	}
	else
	{
		printf("failed to parse controllers from %s\n", file.c_str());
		assert(false);
	}

	return succ;
}

void cBipedController::ResetParams()
{
	cTerrainRLCharController::ResetParams();
	mState = eStateContact;
	mStance = gDefaultStance;

	mPrevCycleTime = 0;
	mPrevDistTraveled.setZero();
	mCurrCycleTime = 0;
	mPrevCOM.setZero();
	mPrevStumbleCount = 0;
	mCurrStumbleCount = 0;
}

bool cBipedController::ParseControllers(const Json::Value& root)
{
	bool succ = true;

	if (!root[gFilesKey].isNull())
	{
		succ &= ParseControllerFiles(root[gFilesKey]);
	}

	if (succ && !root[gActionsKey].isNull())
	{
		succ &= ParseActions(root[gActionsKey]);

		int num_action = GetNumActions();
		if (succ && num_action > 0)
		{
			mDefaultAction = 0;
			if (!root["DefaultAction"].isNull())
			{
				mDefaultAction = root["DefaultAction"].asInt();
				assert(mDefaultAction >= 0 && mDefaultAction < num_action);
			}
		}

		if (!root["EnableGravityCompensation"].isNull())
		{
			mEnableGravityCompensation = root["EnableGravityCompensation"].asBool();
		}

		if (!root["EnableVirtualForces"].isNull())
		{
			mEnableVirtualForces = root["EnableVirtualForces"].asBool();
		}
	}

	return succ;
}

bool cBipedController::ParseControllerFiles(const Json::Value& root)
{
	bool succ = true;
	std::vector<std::string> files;
	assert(root.isArray());

	int num_files = root.size();
	files.resize(num_files);
	for (int f = 0; f < num_files; ++f)
	{
		std::string curr_file = root.get(f, 0).asString();
		ReadParams( curr_file);
	}
	return succ;
}

bool cBipedController::ParseActions(const Json::Value& root)
{
	bool succ = true;
	assert(root.isArray());

	int num_actions = root.size();
	for (int a = 0; a < num_actions; ++a)
	{
		const Json::Value& curr_action = root.get(a, 0);
		tBlendAction action;
		succ &= ParseAction(curr_action, action);
		if (succ)
		{
			action.mID = static_cast<int>(mActions.size());
			mActions.push_back(action);
		}
		else
		{
			succ = false;
			break;
		}
	}

	if (!succ)
	{
		printf("failed to parse actions\n");
		assert(false);
	}
	return succ;
}

bool cBipedController::ParseAction(const Json::Value& root, tBlendAction& out_action) const
{
	if (!root["ParamIdx0"].isNull())
	{
		out_action.mParamIdx0 = root["ParamIdx0"].asInt();
	}
	else
	{
		return false;
	}
	if (!root["ParamIdx1"].isNull())
	{
		out_action.mParamIdx1 = root["ParamIdx1"].asInt();
	}
	else
	{
		return false;
	}
	if (!root["Blend"].isNull())
	{
		out_action.mBlend = root["Blend"].asDouble();
	}
	else
	{
		return false;
	}
	if (!root["Cyclic"].isNull())
	{
		out_action.mCyclic = root["Cyclic"].asBool();
	}
	else
	{
		return false;
	}
	return true;
}

bool cBipedController::HasCtrlParams() const
{
	return mCtrlParams.size() > 0;
}

void cBipedController::UpdateState(double time_step)
{
	const cBipedController::tStateDef& state = GetCurrStateDef();
	bool advance_state = mFirstCycle;

	double trans_time = GetTransTime();
	mPhase += time_step / trans_time;

	if (state.mTransTime)
	{
		if (mPhase >= 1)
		{
			advance_state = true;
		}
	}

	if (state.mTransContact)
	{
		int swing_toe = GetSwingAnkle();
		bool contact = CheckContact(swing_toe);
		if (contact)
		{
			advance_state = true;
		}
	}

	if (advance_state)
	{
		eState next_state = (mFirstCycle) ? eStateContact : state.mNext;
		bool end_step = (next_state == eStateInvalid) || mFirstCycle;

		if (end_step)
		{
			if (!mFirstCycle)
			{
				FlipStance();
			}
			UpdateAction();
			mFirstCycle = false;
		}
		else
		{
			TransitionState(next_state);
		}
	}
}

void cBipedController::UpdateAction()
{
	ParseGround();
	BuildPoliState(mPoliState);

	mIsOffPolicy = true;

	if (HasCommands())
	{
		ProcessCommand(mCurrAction);
	}
	else if (HasNet())
	{
		DecideAction(mCurrAction);
	}
	else if (!IsCurrActionCyclic())
	{
		BuildDefaultAction(mCurrAction);
	}

	ApplyAction(mCurrAction);
}

void cBipedController::UpdateRBDModel()
{
	const Eigen::VectorXd& pose = mChar->GetPose();
	const Eigen::VectorXd& vel = mChar->GetVel();

	mRBDModel->Update(pose, vel);
	cRBDUtil::BuildJacobian(*mRBDModel.get(), mJacobian);
}

void cBipedController::UpdatePDCtrls(double time_step, Eigen::VectorXd& out_tau)
{
	mImpPDCtrl.UpdateControlForce(time_step, out_tau);
}

void cBipedController::UpdateStumbleCounter(double time_step)
{
	bool stumbled = mChar->HasStumbled();
	if (stumbled)
	{
		mCurrStumbleCount += time_step;
	}
}

void cBipedController::UpdateStanceHip()
{
	int stance_hip = GetStanceHip();
	int stance_toe = GetStanceAnkle();
	bool active = IsActiveVFEffector(stance_toe);
	mImpPDCtrl.GetPDCtrl(stance_hip).SetActive(!active);
}

void cBipedController::ApplySwingFeedback(Eigen::VectorXd& out_tau)
{
	double cv = GetCv();
	double cd = GetCd();

	bool first_half = mState == eStateContact || mState == eStateDown;
	cd = (first_half) ? 0 : cd;
	cv = (first_half) ? cv : 0;

	tVector com = mChar->CalcCOM();
	tVector com_vel = mChar->CalcCOMVel();

	int stance_toe_id = GetStanceAnkle();
	int swing_hip_id = GetSwingHip();

	tVector toe_pos = mChar->GetBodyPart(stance_toe_id)->GetPos();
	tVector d = com - toe_pos;
	double d_theta = cd * d[0] + cv * com_vel[0];

	tStateParams params = GetCurrParams();
	double theta0 = params(eStateParamSwingHip);
	double theta_new = theta0 + d_theta;
	Eigen::VectorXd theta_vec(1);
	theta_vec[0] = theta_new;

	mImpPDCtrl.SetTargetTheta(swing_hip_id, theta_vec);
}

void cBipedController::ApplyStanceFeedback(Eigen::VectorXd& out_tau)
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();

	int stance_toe = GetStanceAnkle();
	double hip_tau = 0;

	int stance_hip_id = GetStanceHip();
	int swing_hip_id = GetSwingHip();

	const int root_joints[] =
	{
		cSimBiped::eJointRightHip,
		cSimBiped::eJointLeftHip,
		//cSimBiped::eJointSpine0,
		//cSimBiped::eJointTail0
	};
	const int num_root_joints = sizeof(root_joints) / sizeof(root_joints[0]);

	if (IsActiveVFEffector(stance_toe))
	{
		for (int j = 0; j < num_root_joints; ++j)
		{
			int curr_id = root_joints[j];
			if (curr_id != stance_hip_id)
			{
				int param_offset = cKinTree::GetParamOffset(joint_mat, curr_id);

				// hack assuming revolute joint
				double curr_tau = out_tau(param_offset);
				hip_tau += -curr_tau;
			}
		}

		double target_root_pitch = GetTargetRootPitch();
		double root_pitch = GetRootPitch();
		double root_omega = mChar->GetRootAngVel()[2];

		const cPDController& stance_pd = mImpPDCtrl.GetPDCtrl(stance_hip_id);
		const double kp = stance_pd.GetKp();
		const double kd = stance_pd.GetKd();

		double root_tau = kp * (target_root_pitch - root_pitch) + kd * (-root_omega);
		hip_tau += -root_tau;

		int stance_hip_offset = cKinTree::GetParamOffset(joint_mat, stance_hip_id);
		int stance_hip_size = cKinTree::GetParamSize(joint_mat, stance_hip_id);
		auto stance_tau = out_tau.segment(stance_hip_offset, stance_hip_size);
		stance_tau += hip_tau * Eigen::VectorXd::Ones(stance_hip_size);
	}
}

void cBipedController::ApplyGravityCompensation(Eigen::VectorXd& out_tau)
{
	const double lambda = 0.0001;
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	const Eigen::MatrixXd& body_defs = mChar->GetBodyDefs();
	const Eigen::VectorXd& pose = mChar->GetPose();
	const Eigen::VectorXd& vel = Eigen::VectorXd::Zero(pose.size());

	bool has_support = false;
	Eigen::MatrixXd contact_basis = BuildContactBasis(pose, has_support);

	if (has_support)
	{
		Eigen::VectorXd tau_g;
		cRBDUtil::CalcGravityForce(*mRBDModel.get(), tau_g);
		tau_g = -tau_g;

		int basis_cols = static_cast<int>(contact_basis.cols());

		int root_id = mChar->GetRootID();
		int root_offset = mChar->GetParamOffset(root_id);
		int root_size = mChar->GetParamSize(root_id);

		Eigen::VectorXd b = tau_g.segment(root_offset, root_size);
		Eigen::MatrixXd A = contact_basis.block(root_offset, 0, root_size, basis_cols);

		Eigen::VectorXd W(3);
		W(0) = 0.0001;
		W(1) = 0.0001;
		W(2) = 1;

		Eigen::MatrixXd AtA = A.transpose() * W.asDiagonal() * A;
		Eigen::VectorXd Atb = A.transpose() * W.asDiagonal() * b;
		AtA.diagonal() += lambda * Eigen::VectorXd::Ones(basis_cols);

		Eigen::VectorXd x = (AtA).householderQr().solve(Atb);
		Eigen::VectorXd tau_contact = contact_basis * x;

		tau_g -= tau_contact;
		out_tau += tau_g;
	}
}

void cBipedController::ApplyVirtualForces(Eigen::VectorXd& out_tau)
{
	for (int e = 0; e < gNumEndEffectors; ++e)
	{
		cSimBiped::eJoint joint_id = gEndEffectors[e];
		bool active_effector = IsActiveVFEffector(joint_id);

		int stance_hip_id = GetStanceHip();
		int swing_hip_id = GetSwingHip();
		int root_id = mChar->GetRootID();

		if (active_effector)
		{
			tVector vf = -GetEffectorVF();

			tVector pos = GetEndEffectorContactPos(joint_id);
			cSpAlg::tSpTrans joint_world_trans = cSpAlg::BuildTrans(-pos);

			cSpAlg::tSpVec sp_force = cSpAlg::BuildSV(vf);
			sp_force = cSpAlg::ApplyTransF(joint_world_trans, sp_force);

			const Eigen::MatrixXd& joint_mat = mRBDModel->GetJointMat();
			int curr_id = joint_id;
			while (curr_id != root_id)
			{
				assert(curr_id != cKinTree::gInvalidJointID);
				int offset = cKinTree::GetParamOffset(joint_mat, curr_id);
				int size = cKinTree::GetParamSize(joint_mat, curr_id);
				const auto curr_J = mJacobian.block(0, offset, cSpAlg::gSpVecSize, size);

				out_tau.segment(offset, size) += curr_J.transpose() * sp_force;

				if (curr_id == stance_hip_id)
				{
					int swing_offset = cKinTree::GetParamOffset(joint_mat, swing_hip_id);
					int swing_size = cKinTree::GetParamSize(joint_mat, swing_hip_id);

					out_tau.segment(swing_offset, swing_size) += -(curr_J.transpose() * sp_force);
				}

				curr_id = mRBDModel->GetParent(curr_id);
			}
		}
	}
}

const cBipedController::tStateDef& cBipedController::GetCurrStateDef() const
{
	return gStateDefs[mState];
}

cBipedController::tStateParams cBipedController::GetCurrParams() const
{
	tStateParams params = mCurrAction.mParams.segment(eMiscParamMax + mState * eStateParamMax, eStateParamMax);
	return params;
}

double cBipedController::GetTargetRootPitch() const
{
	tStateParams params = GetCurrParams();
	double pitch = params(eStateParamRootPitch);
	return pitch;
}

double cBipedController::GetRootPitch() const
{
	tVector axis = tVector::Zero();
	double theta = 0;
	mChar->GetRootRotation(axis, theta);

	const tVector& ref_axis = tVector(0, 0, 1, 0);
	if (axis.dot(ref_axis) < 0)
	{
		theta = -theta;
	}
	return theta;
}

void cBipedController::SetStateParams(const tStateParams& params)
{
#if defined(ENABLE_SPINE_CURVE)
	double spine_theta = params(eStateParamSpineCurve);
	for (int i = 0; i < gNumSpineJoints; ++i)
	{
		cSimBiped::eJoint joint = gSpineJoints[i];
		mImpPDCtrl.SetTargetTheta(joint, spine_theta);
	}
#endif

	mImpPDCtrl.SetTargetTheta(GetStanceHip(), params.segment(eStateParamStanceHip, 1));
	mImpPDCtrl.SetTargetTheta(GetStanceKnee(), params.segment(eStateParamStanceKnee, 1));
	// mImpPDCtrl.SetTargetTheta(GetStanceAnkle(), params.segment(eStateParamStanceAnkle, 1));
	mImpPDCtrl.SetTargetTheta(GetSwingHip(), params.segment(eStateParamSwingHip, 1));
	mImpPDCtrl.SetTargetTheta(GetSwingKnee(), params.segment(eStateParamSwingKnee, 1));
	// mImpPDCtrl.SetTargetTheta(GetSwingAnkle(), params.segment(eStateParamSwingAnkle, 1));
}

void cBipedController::SetupPassiveMode()
{
	tStateParams params;
	params.setZero();
	SetStateParams(params);
}

double cBipedController::GetTransTime() const
{
	return mCurrAction.mParams[eMiscParamTransTime];
}

double cBipedController::GetCv() const
{
	return mCurrAction.mParams[eMiscParamCv];
}

double cBipedController::GetCd() const
{
	return mCurrAction.mParams[eMiscParamCd];
}

bool cBipedController::CheckContact(int joint_id) const
{
	assert(joint_id != cSimBiped::eJointInvalid);
	const auto& body_part = mChar->GetBodyPart(joint_id);
	bool contact = body_part->IsInContact();
	return contact;
}

bool cBipedController::IsActiveVFEffector(int joint_id) const
{
	int state = GetState();
	bool valid_effector = (joint_id == GetStanceAnkle()) && (state == eStateContact || state == eStateDown);
	bool active_effector = valid_effector && CheckContact(joint_id);
	return active_effector;
}

tVector cBipedController::GetEffectorVF() const
{
	return tVector(mCurrAction.mParams[eMiscParamForceX], mCurrAction.mParams[eMiscParamForceY], 0, 0);
}

Eigen::MatrixXd cBipedController::BuildContactBasis(const Eigen::VectorXd& pose, bool& out_has_support) const
{
	const int num_basis = 2;
	int rows = static_cast<int>(pose.size());
	const int cols = gNumEndEffectors * num_basis;

	const double cone_theta = M_PI * 0.1;
	double s = std::sin(cone_theta);
	double c = std::cos(cone_theta);

	const Eigen::Matrix<double, 6, num_basis> force_svs
		((Eigen::Matrix<double, 6, num_basis>() <<
			0, 0,
			0, 0,
			0, 0,
			0, 1,
			1, 0,
			0, 0).finished());

	Eigen::MatrixXd contact_basis = Eigen::MatrixXd::Zero(rows, cols);

	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	tVector root_pos = mChar->GetRootPos();

	out_has_support = false;
	for (int e = 0; e < gNumEndEffectors; ++e)
	{
		cSimBiped::eJoint joint_id = gEndEffectors[e];
		bool valid_support = IsActiveVFEffector(joint_id);
		//bool valid_support = CheckContact(joint_id);

		auto curr_block = contact_basis.block(0, e * num_basis, rows, num_basis);
		if (valid_support)
		{
			out_has_support = true;

			tVector pos = GetEndEffectorContactPos(joint_id);
			cSpAlg::tSpTrans joint_world_trans = cSpAlg::BuildTrans(-pos);

			Eigen::Matrix<double, 6, num_basis> force_basis;
			for (int i = 0; i < num_basis; ++i)
			{
				cSpAlg::tSpVec force_sv = force_svs.col(i);
				force_basis.col(i) = cSpAlg::ApplyTransF(joint_world_trans, force_sv);
			}

			int curr_id = joint_id;
			while (curr_id != cKinTree::gInvalidJointID)
			{
				int offset = cKinTree::GetParamOffset(joint_mat, curr_id);
				int size = cKinTree::GetParamSize(joint_mat, curr_id);
				const auto curr_J = mJacobian.block(0, offset, cSpAlg::gSpVecSize, size);

				curr_block.block(offset, 0, size, curr_block.cols()) = curr_J.transpose() * force_basis;
				curr_id = mRBDModel->GetParent(curr_id);
			}
		}
	}

	return contact_basis;
}

const std::string& cBipedController::GetStateName(eState state) const
{
	return gStateDefs[state].mName;
}

const std::string& cBipedController::GetStateParamName(eStateParam param) const
{
	return gStateParamNames[param];
}

void cBipedController::GetOptParams(const Eigen::VectorXd& ctrl_params, Eigen::VectorXd& out_opt_params) const
{
	int num_params = GetNumParams();
	int num_opt_params = GetNumOptParams();
	assert(ctrl_params.size() == num_params);
	assert(gNumOptParamMasks == num_params);

	out_opt_params.resize(num_opt_params);

	int opt_idx = 0;
	for (int i = 0; i < num_params; ++i)
	{
		if (IsOptParam(i))
		{
			out_opt_params[opt_idx] = ctrl_params[i];
			++opt_idx;
		}
	}
	assert(opt_idx == num_opt_params);
}

std::string cBipedController::BuildOptParamsJson(const Eigen::VectorXd& opt_params) const
{
	assert(opt_params.size() == GetNumOptParams());
	Eigen::VectorXd param_buffer = mCurrAction.mParams;
	SetOptParams(opt_params, param_buffer);

	std::string json = "";
	int idx = 0;

	json += "\"" + gMiscParamsKey + "\": \n{\n";
	for (int i = 0; i < eMiscParamMax; ++i)
	{
		if (i != 0)
		{
			json += ",\n";
		}
		json += "\"" + gMiscParamsNames[i] +  "\": " + std::to_string(param_buffer[idx++]);
	}
	json += "\n},\n\n";

	json += "\"" + gStateParamsKey + "\": \n{\n";
	for (int i = 0; i < eStateMax; ++i)
	{
		if (i != 0)
		{
			json += ",\n";
		}

		json += "\"" + gStateDefs[i].mName + "\": \n{\n";
		for (int j = 0; j < eStateParamMax; ++j)
		{
			if (j != 0)
			{
				json += ",\n";
			}
			json += "\"" + gStateParamNames[j] + "\": " + std::to_string(param_buffer[idx++]);
		}
		json += "\n}";
	}
	json += "\n}\n";

	json = "{" + json + "}";
	return json;
}

void cBipedController::BuildStateParamsFromPose(const Eigen::VectorXd& pose, tStateParams& out_params)
{
	const cSimBiped::eJoint ctrl_joints[] =
	{
		cSimBiped::eJointRightHip,
		cSimBiped::eJointRightKnee,
		// cSimBiped::eJointRightAnkle,
		cSimBiped::eJointLeftHip,
		cSimBiped::eJointLeftKnee,
		// cSimBiped::eJointLeftAnkle,
	};
	const int num_ctrl_joints = sizeof(ctrl_joints) / sizeof(ctrl_joints[0]);

	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();

	tQuaternion root_rot = cKinTree::GetRootRot(joint_mat, pose);
	tVector root_axis;
	double root_theta;
	cMathUtil::QuaternionToAxisAngle(root_rot, root_axis, root_theta);
	if (root_axis[2] < 1)
	{
		root_theta = -root_theta;
	}

	out_params(eStateParamRootPitch) = root_theta;
	out_params(eStateParamSpineCurve) = CalcSpineCurve(pose);

	for (int i = 0; i < num_ctrl_joints; ++i)
	{
		tVector axis;
		double tar_theta = 0;
		int joint_id = ctrl_joints[i];
		if (mImpPDCtrl.UseWorldCoord(joint_id))
		{
			cKinTree::CalcJointWorldTheta(joint_mat, pose, joint_id, axis, tar_theta);
		}
		else
		{
			// hack
			Eigen::VectorXd joint_params;
			cKinTree::GetJointParams(joint_mat, pose, joint_id, joint_params);
			tar_theta = joint_params[0];
		}

		tVector ref_axis = tVector(0, 0, 1, 0);
		if (ref_axis.dot(axis) < 0)
		{
			tar_theta = -tar_theta;
		}
		out_params(eStateParamStanceHip + i) = tar_theta;
	}
}

double cBipedController::CalcSpineCurve(const Eigen::VectorXd& pose) const
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();

	double curve = 0;
	for (int i = 0; i < gNumSpineJoints; ++i)
	{
		cSimBiped::eJoint joint = gSpineJoints[i];
		Eigen::VectorXd joint_params;
		cKinTree::GetJointParams(joint_mat, pose, joint, joint_params);
		double theta = joint_params[0]; // hack
		curve += theta;
	}

	curve /= gNumSpineJoints;
	return curve;
}

const Eigen::VectorXd& cBipedController::GetCtrlParams(int ctrl_id) const
{
	return mCtrlParams[ctrl_id];
}

bool cBipedController::IsCurrActionCyclic() const
{
	const tBlendAction& action = mActions[mCurrAction.mID];
	return action.mCyclic;
}

void cBipedController::ApplyAction(int action_id)
{
	cTerrainRLCharController::ApplyAction(action_id);
}

void cBipedController::ApplyAction(const tAction& action)
{
	cTerrainRLCharController::ApplyAction(action);
	TransitionState(eStateContact);
}

void cBipedController::NewCycleUpdate()
{
	cTerrainRLCharController::NewCycleUpdate();
	mPrevCycleTime = mCurrCycleTime;
	mCurrCycleTime = 0;
	mPrevStumbleCount = mCurrStumbleCount;
	mCurrStumbleCount = 0;
	RecordDistTraveled();
	mPrevCOM = mChar->CalcCOM();
}

void cBipedController::BlendCtrlParams(const tBlendAction& action, Eigen::VectorXd& out_params) const
{
	const Eigen::VectorXd& param0 = GetCtrlParams(action.mParamIdx0);
	const Eigen::VectorXd& param1 = GetCtrlParams(action.mParamIdx1);
	double blend = action.mBlend;
	out_params = (1 - blend) * param0 + blend * param1;
}

void cBipedController::PostProcessParams(Eigen::VectorXd& out_params) const
{
	out_params[eMiscParamTransTime] = std::abs(out_params[eMiscParamTransTime]);
	out_params[eMiscParamCv] = std::abs(out_params[eMiscParamCv]);
	out_params[eMiscParamCd] = std::abs(out_params[eMiscParamCd]);
}

void cBipedController::PostProcessAction(tAction& out_action) const
{
	PostProcessParams(out_action.mParams);
}

bool cBipedController::IsOptParam(int param_idx) const
{
	assert(param_idx >= 0 && param_idx < gNumOptParamMasks);
	return gOptParamsMasks[param_idx];
}

void cBipedController::BuildPoliStatePose(Eigen::VectorXd& out_pose) const
{
	cTerrainRLCharController::BuildPoliStatePose(out_pose);
	eStance stance = GetStance();
	if (stance != gDefaultStance)
	{
		FlipPoliPoseStance(out_pose);
	}
}

void cBipedController::BuildPoliStateVel(Eigen::VectorXd& out_vel) const
{
	cTerrainRLCharController::BuildPoliStateVel(out_vel);
	eStance stance = GetStance();
	if (stance != gDefaultStance)
	{
		FlipPoliPoseStance(out_vel);
	}
}

void cBipedController::FlipStance()
{
	SetStance((mStance == eStanceRight) ? eStanceLeft : eStanceRight);
}

void cBipedController::SetStance(eStance stance)
{
	mStance = stance;
	mImpPDCtrl.GetPDCtrl(GetStanceHip()).SetActive(false);
	mImpPDCtrl.GetPDCtrl(GetSwingHip()).SetActive(true);

	TransitionState(GetState(), GetPhase());
}

void cBipedController::FlipPoseStance(Eigen::VectorXd& out_pose) const
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();

	int num_dofs = mChar->GetNumDof();
	int last_body_joint = cSimBiped::eJointHead;
	int start_idx = cKinTree::GetParamOffset(joint_mat, last_body_joint);
	start_idx += cKinTree::GetParamSize(joint_mat, last_body_joint);

	int num_leg_params = (num_dofs - start_idx) / 2;
	for (int i = 0; i < num_leg_params; ++i)
	{
		int idx0 = start_idx + i;
		int idx1 = start_idx + num_leg_params + i;
		double val0 = out_pose(idx0);
		double val1 = out_pose(idx1);
		out_pose(idx0) = val1;
		out_pose(idx1) = val0;
	}
}

void cBipedController::FlipPoliPoseStance(Eigen::VectorXd& out_pose) const
{
	// hack
	// assume that the leg params are all packed at the end of the vector!
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();

	int num_params = static_cast<int>(out_pose.size());
	int num_leg_joints = GetNumJointsPerLeg();
	int num_leg_params = num_leg_joints * gPosDim;

	for (int i = 0; i < num_leg_params; ++i)
	{
		int idx0 = num_params - i - 1;
		int idx1 = num_params - num_leg_params - i - 1;
		double val0 = out_pose(idx0);
		double val1 = out_pose(idx1);
		out_pose(idx0) = val1;
		out_pose(idx1) = val0;
	}
}

int cBipedController::GetNumJointsPerLeg() const
{
	int hip_id = GetStanceHip();
	int toe_id = GetStanceAnkle();
	int num_joints = std::abs(toe_id - hip_id) + 1;
	return num_joints;
}

int cBipedController::GetSwingHip() const
{
	return (GetStance() == eStanceRight) ? cSimBiped::eJointLeftHip : cSimBiped::eJointRightHip;
}

int cBipedController::GetSwingKnee() const
{
	return (GetStance() == eStanceRight) ? cSimBiped::eJointLeftKnee : cSimBiped::eJointRightKnee;
}

int cBipedController::GetSwingAnkle() const
{
	return (GetStance() == eStanceRight) ? cSimBiped::eJointLeftAnkle : cSimBiped::eJointRightAnkle;
}

int cBipedController::GetStanceHip() const
{
	return (GetStance() == eStanceRight) ? cSimBiped::eJointRightHip : cSimBiped::eJointLeftHip;
}

int cBipedController::GetStanceKnee() const
{
	return (GetStance() == eStanceRight) ? cSimBiped::eJointRightKnee : cSimBiped::eJointLeftKnee;
}

int cBipedController::GetStanceAnkle() const
{
	return (GetStance() == eStanceRight) ? cSimBiped::eJointRightAnkle : cSimBiped::eJointLeftAnkle;
}

tVector cBipedController::GetEndEffectorContactPos(int joint_id) const
{
	tVector pos = tVector::Zero();
	const auto& part = mChar->GetBodyPart(joint_id);
	if (part != nullptr)
	{
		const auto& body_defs = mChar->GetBodyDefs();
		tVector size = cKinTree::GetBodySize(body_defs, joint_id);
		pos = part->LocalToWorldPos(tVector(0, -0.5 * size[1], 0, 0));
	}
	else
	{
		pos = mRBDModel->CalcJointWorldPos(joint_id);
	}
	return pos;
}

void cBipedController::RecordDistTraveled()
{
	tVector com = mChar->CalcCOM();
	mPrevDistTraveled = com - mPrevCOM;
}

int cBipedController::PopCommand()
{
	int cmd = mCommands.top();
	mCommands.pop();
	return cmd;
}

bool cBipedController::HasCommands() const
{
	return !mCommands.empty();
}

void cBipedController::ClearCommands()
{
	while (!mCommands.empty())
	{
		mCommands.pop();
	}
}

void cBipedController::ProcessCommand(tAction& out_action)
{
	int a_id = PopCommand();
	BuildBaseAction(a_id, out_action);
}

void cBipedController::BuildBaseAction(int action_id, tAction& out_action) const
{
	const tBlendAction& blend = mActions[action_id];
	out_action.mID = blend.mID;
	BlendCtrlParams(blend, out_action.mParams);
}
