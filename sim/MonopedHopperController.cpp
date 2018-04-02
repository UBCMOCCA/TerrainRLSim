#include "MonopedHopperController.h"
#include <iostream>
#include <ctime>
#include "util/json/json.h"

#include "sim/SimCharacter.h"
#include "sim/RBDUtil.h"
#include "util/FileUtil.h"

//#define MonopedHopper_CTRL_PROFILER
#define ENBLE_MAX_COORD_POSE

const cMonopedHopperController::tStateDef gStateDefs[cMonopedHopperController::eStateMax] =
{
	{
		"Descent",
		false,								// mTransTime;
		cSimMonopedHopper::eJointKnee,				// mTransFootContact;
		cMonopedHopperController::eStateDownStance		// mNext;
	},
	{
		"DownStance",
		false,								// mTransTime;
		cSimMonopedHopper::eJointInvalid,				// mTransContact;
		cMonopedHopperController::eStateUpStance	// mNext;
	},
	{
		"UpStance",
		false,								// mTransTime;
		cSimMonopedHopper::eJointKnee,				// mTransFootContact;
		cMonopedHopperController::eStateAscent		// mNext;
	},
	{
		"Ascent",
		false,								// mTransTime;
		cSimMonopedHopper::eJointKnee,				// mTransFootContact;
		cMonopedHopperController::eStateDescent		// mNext;
	},
};

const std::string gMiscParamsNames[cMonopedHopperController::eMiscParamMax] = {
	"MagicGain"
};

const std::string gStateParamNames[cMonopedHopperController::eStateParamMax] = {
	"Hip",
	"Knee"
};

const cSimMonopedHopper::eJoint gEndEffectors[] = {
	cSimMonopedHopper::eJointKnee,
};
const int gNumEndEffectors = sizeof(gEndEffectors) / sizeof(gEndEffectors[0]);

const cSimMonopedHopper::eJoint gSpineJoints[] = {
	cSimMonopedHopper::eJointHip,
	cSimMonopedHopper::eJointKnee
};
const int gNumSpineJoints = sizeof(gSpineJoints) / sizeof(gSpineJoints[0]);


const std::string gMiscParamsKey = "MiscParams";
const std::string gStateParamsKey = "StateParams";
const std::string gControllersKey = "Controllers";
const std::string gFilesKey = "Files";
const std::string gActionsKey = "Actions";

const bool gOptParamsMasks[] =
{
	true,	//MagicGain

	//Descent
	true,	//Hip
	true,	//Knee

	//DownStance
	true,	//Hip
	true,	//Knee

	//UpStance
	true,	//Hip
	true,	//Knee

	//Ascent
	true,	//Hip
	true,	//Knee
};
const int gNumOptParamMasks = sizeof(gOptParamsMasks) / sizeof(gOptParamsMasks[0]);
const int gPosDim = cKinTree::gPosDim - 1;

cMonopedHopperController::cMonopedHopperController() : cTerrainRLCharController()
{
	mDefaultAction = gInvalidIdx;
	mState = eStateDescent;
	mEnableGravityCompensation = true;
}

cMonopedHopperController::~cMonopedHopperController()
{
}

void cMonopedHopperController::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
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
		// mImpPDCtrl.Init(mChar, pd_params);

		succ = LoadControllers(param_file);
		TransitionState(eStateDescent);
	}

	mValid = succ;
	if (!mValid)
	{
		printf("Failed to initialize MonopedHopper controller\n");
		mValid = false;
	}
}

void cMonopedHopperController::Reset()
{
	cTerrainRLCharController::Reset();
	ClearCommands();
	mImpPDCtrl.Reset();
	mPrevCOM = mChar->CalcCOM();
}

void cMonopedHopperController::Clear()
{
	cTerrainRLCharController::Clear();
	mImpPDCtrl.Clear();
	mCtrlParams.clear();
	ClearCommands();

	mRBDModel.reset();
	mDefaultAction = gInvalidIdx;
}

void cMonopedHopperController::Update(double time_step)
{
	cTerrainRLCharController::Update(time_step);
}

void cMonopedHopperController::UpdateCalcTau(double time_step, Eigen::VectorXd& out_tau)
{
	if (mMode == eModeActive)
	{
#if defined(MonopedHopper_CTRL_PROFILER)
		static int time_count = 0;
		static double avg_time = 0;
		std::clock_t total_time_beg = std::clock();
#endif
		mCurrCycleTime += time_step;
		int num_dof = mChar->GetNumDof();
		out_tau = Eigen::VectorXd::Zero(num_dof);

		UpdateRBDModel();
		UpdateState(time_step);
		ApplyFeedback(out_tau);

		UpdatePDCtrls(time_step);

		ApplyGravityCompensation(out_tau);
		ApplyVirtualForces(out_tau);

#if defined(MonopedHopper_CTRL_PROFILER)
		std::clock_t total_time_end = std::clock();
		double time_elapsed = static_cast<double>(total_time_end - total_time_beg) / CLOCKS_PER_SEC;
		++time_count;
		avg_time = avg_time * ((time_count - 1.0) / time_count) + time_elapsed / time_count;
		printf("MonopedHopper Ctrl Update Time: %.8f, count: %i\n", avg_time, time_count);
#endif
	}
}

void cMonopedHopperController::UpdateApplyTau(const Eigen::VectorXd& tau)
{
	mTau = tau;
	mChar->ApplyControlForces(tau);
}

void cMonopedHopperController::SeCtrlStateParams(eState state, const tStateParams& params)
{
	mCurrAction.mParams.segment(eMiscParamMax + state * eStateParamMax, eStateParamMax) = params;
}

void cMonopedHopperController::TransitionState(int state)
{
	cTerrainRLCharController::TransitionState(state);
}

void cMonopedHopperController::TransitionState(int state, double phase)
{
	assert(state >= 0 && state < eStateMax);
	cTerrainRLCharController::TransitionState(state, phase);
	tStateParams params = GetCurrParams();
	SetStateParams(params);
}


int cMonopedHopperController::GetNumStates() const
{
	return eStateMax;
}

void cMonopedHopperController::SetMode(eMode mode)
{
	cTerrainRLCharController::SetMode(mode);

	if (mMode == eModePassive)
	{
		SetupPassiveMode();
	}
}

void cMonopedHopperController::CommandAction(int action_id)
{
	int num_actions = GetNumActions();
	if (action_id >= 0 && action_id < num_actions)
	{
		mCommands.push(action_id);
	}
	else
	{
		assert(false); // invalid action
	}
}

void cMonopedHopperController::CommandRandAction()
{
	int num_actions = GetNumActions();
	int a = cMathUtil::RandInt(0, num_actions);
	CommandAction(a);

#if defined (ENABLE_DEBUG_PRINT)
	printf("rand action: %i\n", a);
#endif
}

int cMonopedHopperController::GetDefaultAction() const
{
	return mDefaultAction;
}

void cMonopedHopperController::SetDefaultAction(int action_id)
{
	if (action_id >= 0 && action_id < GetNumActions())
	{
		mDefaultAction = action_id;
	}
}

int cMonopedHopperController::GetNumActions() const
{
	return static_cast<int>(mActions.size());
}

void cMonopedHopperController::BuildCtrlOptParams(int ctrl_params_idx, Eigen::VectorXd& out_params) const
{
	GetOptParams(mCtrlParams[ctrl_params_idx], out_params);
}

void cMonopedHopperController::SetCtrlParams(int ctrl_params_id, const Eigen::VectorXd& params)
{
	assert(params.size() == GetNumParams());
	Eigen::VectorXd& ctrl_params = mCtrlParams[ctrl_params_id];
	ctrl_params = params;
	PostProcessParams(ctrl_params);

	const tBlendAction& curr_action = mActions[mCurrAction.mID];
	if (curr_action.mParamIdx0 == ctrl_params_id
		|| curr_action.mParamIdx1 == ctrl_params_id)
	{
		ApplyAction(mCurrAction);
	}
}

void cMonopedHopperController::SetCtrlOptParams(int ctrl_params_id, const Eigen::VectorXd& params)
{
	assert(params.size() == GetNumOptParams());
	Eigen::VectorXd& ctrl_params = mCtrlParams[ctrl_params_id];
	ctrl_params.segment(0, GetNumOptParams()) = params;
	PostProcessParams(ctrl_params);

	const tBlendAction& curr_action = mActions[mCurrAction.mID];
	if (curr_action.mParamIdx0 == ctrl_params_id
		|| curr_action.mParamIdx1 == ctrl_params_id)
	{
		ApplyAction(mCurrAction);
	}
}

void cMonopedHopperController::BuildActionOptParams(int action_id, Eigen::VectorXd& out_params) const
{
	assert(action_id >= 0 && action_id < GetNumActions());
	const tBlendAction& action = mActions[action_id];
	Eigen::VectorXd params;
	BlendCtrlParams(action, params);
	GetOptParams(params, out_params);
}

int cMonopedHopperController::GetNumParams() const
{
	int num_params = eMiscParamMax + eStateMax * eStateParamMax;
	return num_params;
}

int cMonopedHopperController::GetNumOptParams() const
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

void cMonopedHopperController::FetchOptParamScale(Eigen::VectorXd& out_scale) const
{
	int num_opt_params = GetNumOptParams();
	int num_params = GetNumParams();

	const double angle_scale = M_PI / 10;
	const double gain_scale = 0.1;

	out_scale.resize(num_params);

	Eigen::VectorXd param_buffer = Eigen::VectorXd::Ones(num_params);

	int idx = 0;
	for (int i = 0; i < eMiscParamMax; ++i)
	{
		param_buffer[idx] = gain_scale;
		++idx;
	}

	for (int i = 0; i < eStateMax; ++i)
	{
		for (int j = 0; j < eStateParamMax; ++j)
		{
			param_buffer[idx] = angle_scale;
			++idx;
		}
	}

	assert(idx == num_params);
	GetOptParams(param_buffer, out_scale);
}

void cMonopedHopperController::OutputOptParams(const std::string& file, const Eigen::VectorXd& params) const
{
	cController::OutputOptParams(file, params);
}

void cMonopedHopperController::OutputOptParams(FILE* f, const Eigen::VectorXd& params) const
{
	std::string opt_param_json = BuildOptParamsJson(params);
	fprintf(f, "%s\n", opt_param_json.c_str());
}

void cMonopedHopperController::ReadParams(std::ifstream& f_stream)
{
	Json::Reader reader;
	Json::Value root;
	bool succ = reader.parse(f_stream, root);
	if (succ)
	{
		double trans_time = 0;
		Eigen::VectorXd param_vec = Eigen::VectorXd::Zero(GetNumOptParams());
		int idx = 0;
		// grab the transition time first
		if (!root[gMiscParamsKey].isNull())
		{
			Json::Value misc_params = root[gMiscParamsKey];
			// trans_time = misc_params[gMiscParamsNames[eMiscParamTransTime]].asDouble();
			// param_vec[0] = trans_time;

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
		mCtrlParams.push_back(param_vec);
	}
}

void cMonopedHopperController::ReadParams(const std::string& file)
{
	cTerrainRLCharController::ReadParams(file);
}

void cMonopedHopperController::SetParams(const Eigen::VectorXd& params)
{
	assert(params.size() == GetNumParams());
	mCurrAction.mParams = params;
	PostProcessParams(mCurrAction.mParams);
}

void cMonopedHopperController::BuildOptParams(Eigen::VectorXd& out_params) const
{
	GetOptParams(mCurrAction.mParams, out_params);
}

void cMonopedHopperController::SetOptParams(const Eigen::VectorXd& opt_params)
{
	SetOptParams(opt_params, mCurrAction.mParams);
}

void cMonopedHopperController::SetOptParams(const Eigen::VectorXd& opt_params, Eigen::VectorXd& out_params) const
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
	assert(opt_idx == GetNumOptParams());

	PostProcessParams(out_params);
}

void cMonopedHopperController::BuildFromMotion(int ctrl_params_idx, const cMotion& motion)
{
	assert(false); // need to check if this works
	assert(motion.IsValid());
	double dur = motion.GetDuration();
	int num_states = GetNumStates();
	double trans_time = dur / num_states;
	// SetTransTime(trans_time);

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

double cMonopedHopperController::CalcReward() const
{
	const double target_vel = 3.00;
	double reward = 0;

	bool fallen = mChar->HasFallen();
	if (!fallen)
	{
		double cycle_time = GetPrevCycleTime();
		double avg_vel = mPrevDistTraveled[0] / cycle_time;
		double vel_err = target_vel - avg_vel;
		double vel_gamma = 0.5;
		double vel_reward = std::exp(-vel_gamma * vel_err * vel_err);

		reward += vel_reward;
	}
	// std::cout << "Reward: " << reward << std::endl;
	return reward;
}


double cMonopedHopperController::GetPrevCycleTime() const
{
	return mPrevCycleTime;
}

const tVector& cMonopedHopperController::GetPrevDistTraveled() const
{
	return mPrevDistTraveled;
}

void cMonopedHopperController::ResetParams()
{
	cTerrainRLCharController::ResetParams();
	mState = eStateDescent;
	mPrevCycleTime = 0;
	mCurrCycleTime = 0;
	mPrevCOM.setZero();
	mPrevStumbleCount = 0;
	mCurrStumbleCount = 0;
	_previousBody_Y = 0;
	_errors = Eigen::VectorXd::Zero(cSimMonopedHopper::eJointMax);
	_desired_angles = Eigen::VectorXd::Zero(cSimMonopedHopper::eJointMax);
}

bool cMonopedHopperController::LoadControllers(const std::string& file)
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
			succ &= ParseControllerFiles(ctrl_root);

			if (succ)
			{
				succ &= ParseActions(ctrl_root);
			}
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

bool cMonopedHopperController::ParseControllers(const Json::Value& root)
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
	}

	return succ;
}

std::string cMonopedHopperController::BuildOptParamsJson(const Eigen::VectorXd& opt_params) const
{
	assert(opt_params.size() == GetNumOptParams());
	Eigen::VectorXd param_buffer = mCurrAction.mParams;
	SetOptParams(opt_params, param_buffer);
	std::string json = BuildParamsJson(param_buffer);
	return json;
}

std::string cMonopedHopperController::BuildParamsJson(const Eigen::VectorXd& params) const
{
	std::string json = "";
	int idx = 0;

	json += "\"" + gMiscParamsKey + "\": \n{\n";
	for (int i = 0; i < eMiscParamMax; ++i)
	{
		if (i != 0)
		{
			json += ",\n";
		}
		json += "\"" + gMiscParamsNames[i] + "\": " + std::to_string(params[idx++]);
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
			json += "\"" + gStateParamNames[j] + "\": " + std::to_string(params[idx++]);
		}
		json += "\n}";
	}
	json += "\n}\n";

	json = "{" + json + "}";
	return json;
}

void cMonopedHopperController::DebugPrintAction(const tAction& action) const
{
	printf("Action ID: %i\n", action.mID);
	std::string opt_param_json = BuildParamsJson(action.mParams);
	printf("Action params: \n %s\n", opt_param_json.c_str());
}


bool cMonopedHopperController::ParseControllerFiles(const Json::Value& root)
{
	bool succ = true;
	if (!root[gFilesKey].isNull())
	{
		std::vector<std::string> files;
		Json::Value files_json = root[gFilesKey];
		int num_files = files_json.size();
		files.resize(num_files);
		for (int f = 0; f < num_files; ++f)
		{
			std::string curr_file = files_json.get(f, 0).asString();
			std::cout << "Parsing Controller file: " << mChar->getRelativeFilePath() + curr_file << std::endl;
			ReadParams( mChar->getRelativeFilePath() + curr_file);
		}
	}
	return succ;
}

bool cMonopedHopperController::ParseActions(const Json::Value& root)
{
	bool succ = true;
	if (!root[gActionsKey].isNull())
	{
		Json::Value actions_json = root[gActionsKey];
		int num_actions = actions_json.size();

		for (int a = 0; a < num_actions; ++a)
		{
			const Json::Value& curr_action = actions_json.get(a, 0);
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
	}

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

	if (!succ)
	{
		printf("failed to parse actions\n");
		assert(false);
	}
	return succ;
}

bool cMonopedHopperController::ParseAction(const Json::Value& root, tBlendAction& out_action) const
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

bool cMonopedHopperController::HasCtrlParams() const
{
	return mCtrlParams.size() > 0;
}

void cMonopedHopperController::UpdateState(double time_step)
{
	const cMonopedHopperController::tStateDef& state = GetCurrStateDef();
	bool advance_state = false;

	// double trans_time = GetTransTime();
	// _desired_angles(2) = 0.0;
	// mPhase += time_step / trans_time;
	mPhase += 1;

	std::shared_ptr<cSimObj>& root = this->mChar->GetBodyPart(cSimMonopedHopper::eJointRoot);
	// root->GetSimBody()->setDamping(0.0, 0.2); // joint friction
	double Body_Y = root->GetLinearVelocity()(1);

	// std::cout << "x pos " << root->GetLinearVelocity()(0) << std::endl;

	std::shared_ptr<cSimObj>& thigh = this->mChar->GetBodyPart(cSimMonopedHopper::eJointHip);
	// thigh->GetSimBody()->setDamping(0.0, 0.2); // joint friction

	std::shared_ptr<cSimObj>& chin = this->mChar->GetBodyPart(cSimMonopedHopper::eJointKnee);
	// chin->GetSimBody()->setDamping(0.15, 0.0); // joint friction
	cJoint& hip = this->mChar->GetJoint(cSimMonopedHopper::eJointHip);
	cJoint& knee = this->mChar->GetJoint(cSimMonopedHopper::eJointKnee);
	/*
	if (state.mTransTime)
	{
		if (mPhase >= 1)
		{
			advance_state = true;
		}
	}*/

	// std::cout << "Previous distance traveled: " << mPrevDistTraveled << std::endl;
	if (state.mName == "DownStance")
	{
		std::shared_ptr<cSimObj>& chin = this->mChar->GetBodyPart(cSimMonopedHopper::eJointKnee);
		//chin->GetSimBody()->setDamping(10.3, 0.0);
		// bool contact = CheckContact(state.mTransContact);
		// std::cout << "previous vel_y: " << _previousBody_Y << " current vel_y " << Body_Y << std::endl;
		{ // starting to fall
		if ((_previousBody_Y < 0.0) && (Body_Y > 0.0))
			advance_state = true;
		}
	}

	if ( state.mName == "UpStance" )
	{
		// _desired_angles(2) = 0.15;
		std::shared_ptr<cSimObj>& chin = this->mChar->GetBodyPart(cSimMonopedHopper::eJointKnee);
		//chin->GetSimBody()->setDamping(0.2, 0.0);
		tVector pos = chin->GetPos();
		bool contact = CheckContact(state.mTransContact);
		tVector out_axis;
		double out_theta;
		chin->GetRotation(out_axis, out_theta);
		auto& ching_rb = chin->GetSimBody();
		btVector3 center;
		btScalar radius;
		ching_rb->getCollisionShape()->getBoundingSphere(center, radius);
		double c_H = radius + 0.01;
		// std::cout << "chin y: " << pos(1) << std::endl;
		// std::cout << "chin pos_y: " << pos(1) << " chin_t_component " << (c_H*cos(out_theta)) << std::endl;
		// bool contact = CheckContact(state.mTransContact);
		double d_threshold = c_H*cos(out_theta);
		double pris_delta = knee.CalcDisplacementPrismatic();
		//if ((pos(1) > d_threshold))
		if (!contact && (pos(1) > d_threshold))
		// if (pris_delta > 0.0f)
		{ // No longer touching ground
			// std::cout << "chin pos_y: " << pos(1) << " chin_t_component " << (c_H*cos(pris_delta)) << std::endl;
			advance_state = true;
		}
	}

	// std::cout << "Previous distance traveled: " << mPrevDistTraveled << std::endl;
	if (state.mName == "Ascent")
	{
		std::shared_ptr<cSimObj>& chin = this->mChar->GetBodyPart(cSimMonopedHopper::eJointKnee);
		//chin->GetSimBody()->setDamping(0.2, 0.0);
		tVector pos = chin->GetPos();
		// bool contact = CheckContact(state.mTransContact);
		tVector out_axis;
		double out_theta;
		chin->GetRotation(out_axis, out_theta);
		auto& ching_rb = chin->GetSimBody();
		btVector3 center;
		btScalar radius;
		ching_rb->getCollisionShape()->getBoundingSphere(center, radius);
		double c_H = radius + 0.1;
		// std::cout << "chin y: " << pos(1) << std::endl;
		// std::cout << "chin pos_y: " << pos(1) << " chin_t_component " << (c_H*cos(out_theta)) << std::endl;
		// bool contact =  ((pos(1) < (c_H*cos(out_theta))));
		bool contact = CheckContact(state.mTransContact);
		if (contact)
		{
			TransitionState(cMonopedHopperController::eStateUpStance);
		}
		else if ((_previousBody_Y > 0.0) && (Body_Y < 0.0) && (!contact))
		{ // starting to fall
			advance_state = true;
		}
	}

	if (state.mName == "Descent")
	{
		//chin->GetSimBody()->setDamping(0.2, 0.0);
		tVector pos = chin->GetPos();
		// bool contact = CheckContact(state.mTransContact);
		tVector out_axis;
		double out_theta;
		chin->GetRotation(out_axis, out_theta);
		auto& ching_rb = chin->GetSimBody();
		btVector3 center;
		btScalar radius;
		ching_rb->getCollisionShape()->getBoundingSphere(center, radius);
		double c_H = radius + 0.0;
		// std::cout << "chin y: " << pos(1) << std::endl;
		// std::cout << "chin pos_y: " << pos(1) << " chin_t_component " << (c_H*cos(out_theta)) << std::endl;
		// bool contact =  ((pos(1) < (c_H*cos(out_theta))));
		bool contact = CheckContact(state.mTransContact);
		if ( (contact) )
		{ // starting to fall
			advance_state = true;
		}
	}

	if (advance_state)
	{
		eState next_state = state.mNext;
		std::cout << "Was in state: " << state.mName << " Now in state: " << next_state << std::endl;
		bool end_step = (next_state == eStateDownStance);
		if (end_step)
		{
			// std::cout << "updating action " << std::endl;
			UpdateAction();
		}
		else
		{
			TransitionState(next_state);
		}
	}

	_previousBody_Y = Body_Y;
}

void cMonopedHopperController::UpdateAction()
{
	mGroundSampleTrans = BuildGroundSampleTrans();
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

	// std::cout << "Current params: " << GetCurrParams() << std::endl;
}

void cMonopedHopperController::UpdateRBDModel()
{
	const Eigen::VectorXd& pose = mChar->GetPose();
	const Eigen::VectorXd& vel = mChar->GetVel();

	mRBDModel->Update(pose, vel);
	cRBDUtil::BuildJacobian(*mRBDModel.get(), mJacobian);
}

void cMonopedHopperController::UpdatePDCtrls(double time_step)
{
	mImpPDCtrl.Update(time_step);
}

void cMonopedHopperController::UpdateStumbleCounter(double time_step)
{
	bool stumbled = mChar->HasStumbled();
	if (stumbled)
	{
		mCurrStumbleCount += time_step;
	}
}

double cMonopedHopperController::GetRootPitch() const
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

void cMonopedHopperController::ApplyFeedback(Eigen::VectorXd& out_tau)
{
	const cMonopedHopperController::tStateDef& state = GetCurrStateDef();
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	std::shared_ptr<cSimObj>& thigh = this->mChar->GetBodyPart(cSimMonopedHopper::eJointHip);
	std::shared_ptr<cSimObj>& chin = this->mChar->GetBodyPart(cSimMonopedHopper::eJointKnee);
	tVector out_axis;
	double out_theta;

	cJoint& knee = this->mChar->GetJoint(cSimMonopedHopper::eJointKnee);
	double pris_delta = knee.CalcDisplacementPrismatic();
	// std::cout << "knee displacement " << pris_delta << std::endl;
	double legLength = 0.678 + pris_delta;
	double hip_tau = 0;
	double magicGain = this->GetGetMagicGain();

	thigh->GetRotation(out_axis, out_theta) ;
	out_theta *= out_axis(2);
	std::shared_ptr<cSimObj>& root = this->mChar->GetBodyPart(cSimMonopedHopper::eJointRoot);
	tVector position = root->GetPos();
	tVector root_out_axis;
	double root_out_theta;
	root->GetRotation(root_out_axis, root_out_theta) ;
	root_out_theta *= root_out_axis(2);
	int param_id = 0; // the hip
	double default_theta = GetCurrParams()[param_id];
	// default_theta = -0.2; // TODO fix parameter setting, actions changes back to default of 0.
	if (state.mName == "DownStance" || state.mName == "UpStance"
			// ||
			// (chin->IsInContact())
			)
	{ // foot on ground

		double footX = legLength*sin(out_theta);
		double delx = position(0) - footX;
		double dely = position(1);
		double corrected_theta = atan2(-delx,dely) * out_axis(2) + root_out_theta;
		// std::cout << "Standing: root theta " << root_out_theta << "root axis " << root_out_axis.transpose() << " corrected theta for thigh: " << corrected_theta << " around axis " << out_axis.transpose() << std::endl;
		// mImpPDCtrl.SetTargetTheta(cSimMonopedHopper::eJointHip, corrected_theta);
		// _desired_angles(1)= (-root_out_theta);
		// std::cout << "hip desire angle " << out_theta << std::endl;
		_desired_angles(1)= (out_theta ) + default_theta;

		int stance_hip_id = cSimMonopedHopper::eJointHip;
		double target_root_pitch = 0.0; // GetTargetRootPitch();
		double root_pitch = GetRootPitch();
		double root_omega = mChar->GetRootAngVel()[2];

		const cPDController& stance_pd = mImpPDCtrl.GetPDCtrl(stance_hip_id);
		const double kp = stance_pd.GetKp();
		const double kd = stance_pd.GetKd();

		double root_tau = kp * (target_root_pitch - root_pitch) + kd * (-root_omega);
		hip_tau += -root_tau * magicGain;
		int stance_hip_offset = cKinTree::GetParamOffset(joint_mat, stance_hip_id);
		int stance_hip_size = cKinTree::GetParamSize(joint_mat, stance_hip_id);
		auto stance_tau = out_tau.segment(stance_hip_offset, stance_hip_size);
		stance_tau += hip_tau * Eigen::VectorXd::Ones(stance_hip_size);

	}
	else
	{ // foot in air
		// Controller is getting into thew wrongf state.b
		// std::cout << "Current Magic Gain: " << magicGain << std::endl;
		// double corrected_theta = ((root->GetLinearVelocity()(0)*magicGain) + root_out_theta);
		double corrected_theta = ((root->GetLinearVelocity()(0)*magicGain));
		// std::cout << "Lin velociy: " << root->GetLinearVelocity()(0) << std::endl;
		// double corrected_theta = ((this->mChar->GetVel0()(0)*magicGain));
	// mImpPDCtrl.SetTargetTheta(cSimMonopedHopper::eJointHip, corrected_theta);
		// std::cout << "Flying: root theta " << root_out_theta << "root axis " << root_out_axis.transpose() << " corrected theta for thigh: " << corrected_theta << " around axis " << out_axis.transpose() << std::endl;
		root_out_theta *= -1.0f;
		// std::cout << "no contact desired hip angle: " << default_theta << std::endl;
		_desired_angles(1)= corrected_theta + (root_out_theta) + default_theta;
	}
	param_id = 1;
	default_theta = GetCurrParams()[param_id];
	_desired_angles(2) = default_theta;

}

void cMonopedHopperController::ApplyGravityCompensation(Eigen::VectorXd& out_tau)
{
#if defined(MonopedHopper_CTRL_PROFILER)
	static int time_count = 0;
	static double avg_time = 0;
	std::clock_t total_time_beg = std::clock();
#endif

	const double lambda = 0.0001;
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	const Eigen::MatrixXd& body_defs = mChar->GetBodyDefs();
	Eigen::VectorXd pose = mChar->GetPose();
	Eigen::VectorXd vel = Eigen::VectorXd::Zero(pose.size());

	bool has_support = false;
	// This does not work well for me because the leg starts to loose contact because of noise
	// Eigen::MatrixXd contact_basis = BuildContactBasis(pose, has_support);
	const cMonopedHopperController::tStateDef& state = GetCurrStateDef();

	std::shared_ptr<cSimObj>& chin = this->mChar->GetBodyPart(cSimMonopedHopper::eJointKnee);
	tVector pos = chin->GetPos();
	tVector out_axis;
	double out_theta;
	chin->GetRotation(out_axis, out_theta) ;
	auto& ching_rb = chin->GetSimBody();
	btVector3 center;
	btScalar radius;
	ching_rb->getCollisionShape()->getBoundingSphere(center, radius);
	double c_H = radius;


	// if (state.mName == "DownStance" || state.mName == "UpStance")
	// if (has_support)
	if (chin->IsInContact())
	//if ( pos(1) < (c_H*cos(out_theta)) ) // foot touching ground
	{
		std::shared_ptr<cSimObj>& thigh = this->mChar->GetBodyPart(cSimMonopedHopper::eJointHip);
		std::shared_ptr<cSimObj>& root = this->mChar->GetBodyPart(cSimMonopedHopper::eJointRoot);
		
		Eigen::VectorXd tau_g;
		cRBDUtil::CalcGravityForce(*mRBDModel.get(), tau_g);
		tau_g = -tau_g;
		
		tVector force = (root->GetMass() + 0.5) * mGravity;

		tMatrix trans = thigh->GetWorldTransform();
		tVector thigh_dir = tVector(0, 1, 0, 0);
		thigh_dir = trans * thigh_dir;
		
		double leg_force = thigh_dir.dot(force);
		tau_g.setZero();
		
		int leg_offset = cKinTree::GetParamOffset(joint_mat, cSimMonopedHopper::eJointKnee);
		int leg_size = cKinTree::GetParamSize(joint_mat, cSimMonopedHopper::eJointKnee);
		tau_g.segment(leg_offset, leg_size) = leg_force * Eigen::VectorXd::Ones(leg_size);

		out_tau += tau_g;
		
#if defined(MonopedHopper_CTRL_PROFILER)
		std::clock_t total_time_end = std::clock();
		double time_elapsed = static_cast<double>(total_time_end - total_time_beg) / CLOCKS_PER_SEC;
		++time_count;
		avg_time = avg_time * ((time_count - 1.0) / time_count) + time_elapsed / time_count;
		printf("Gravity Comp Time: %.5f, count: %i\n", avg_time, time_count);
#endif
	}
}

void cMonopedHopperController::ApplyVirtualForces(Eigen::VectorXd& out_tau)
{
	// std::cout << "Desired Angles: " << _desired_angles.transpose() << std::endl;
	std::shared_ptr<cSimObj>& thigh = this->mChar->GetBodyPart(cSimMonopedHopper::eJointHip);

	cJoint& hip = this->mChar->GetJoint(cSimMonopedHopper::eJointHip);
	cJoint& knee = this->mChar->GetJoint(cSimMonopedHopper::eJointKnee);
	cPDController& pd_hip = mImpPDCtrl.GetPDCtrl(cSimMonopedHopper::eJointHip);
	cPDController& pd_knee = mImpPDCtrl.GetPDCtrl(cSimMonopedHopper::eJointKnee);
	// std::cout << "Hip Kp: " << pd_hip.GetKp() << " Kd: " << pd_hip.GetKd() << " torqueLimit "<< pd_hip.GetTorqueLimit() << std::endl;
	float Kp = pd_hip.GetKp(), Kd = pd_hip.GetKd();

	// hack should not access this
	double world_scale = mChar->GetWorld()->GetScale();

	float max_torque_hip = pd_hip.GetTorqueLimit();
	float max_torque = pd_hip.GetTorqueLimit();
	max_torque_hip *= world_scale * world_scale;
	max_torque *= world_scale * world_scale;

	tVector out_axis;
	double out_theta;
	hip.CalcRotationRevolute(out_axis, out_theta);
	// thigh->GetRotation(out_axis, out_theta) ;
	out_theta *= out_axis(2);
	float error = _desired_angles(1) - (out_theta);
	float error_dt = error - _errors(1);
	_errors(1) = error;
	/*
	out_tau(3) += ((error * Kp) + (error_dt * Kd));
	if ( (out_tau(3)) > max_torque )
	{
		// std::cout << "Exceded max torque: " << out_tau(3) << std::endl;
		out_tau(3) = max_torque ;
	}
	if ( out_tau(3) < -max_torque )
	{
		// std::cout << "Exceded max torque: " << out_tau(3) << std::endl;
		out_tau(3) = -max_torque ;
	}
	std::shared_ptr<cSimObj>& chin = this->mChar->GetBodyPart(cSimMonopedHopper::eJointKnee);
	if (!chin->IsInContact())
		//if ( pos(1) < (c_H*cos(out_theta)) ) // foot touching ground
	{
		out_tau *= 0.0;
	}
	std::cout << "Hip angle: " << out_theta << " out axis: " << out_axis.transpose() << " error " << error << " hip_torque " << out_tau(3) << std::endl;
	*/
	/*
	double pris_delta = knee.CalcDisplacementPrismatic();
	error = _desired_angles(2) - pris_delta;
	error_dt = error - _errors(2);
	_errors(2) = error;
	Kp = pd_knee.GetKp(), Kd = pd_knee.GetKd();
	double max_torque_knee = pd_knee.GetTorqueLimit();
	max_torque = pd_knee.GetTorqueLimit();
	max_torque_knee *= world_scale;
	max_torque *= world_scale;

	out_tau(4) += -((error * Kp) + (error_dt * Kd));
	if ( (out_tau(4)) > max_torque )
	{
		// std::cout << "Exceded max knee torque: " << out_tau(4) << std::endl;
		out_tau(4) = max_torque ;
	}
	if ( out_tau(4) < -max_torque )
	{
		// std::cout << "Exceded max knee torque: " << out_tau(4) << std::endl;
		out_tau(4) = -max_torque ;
	}
	*/
	// std::cout << "desired values: " << _desired_angles.transpose() << std::endl;
	/*
	btHingeConstraint * btHip = static_cast<btHingeConstraint*>(this->mChar->GetJoint(cSimMonopedHopper::eJointHip).GetConstraintHandle().mCons);
	btSliderConstraint * btKnee = static_cast<btSliderConstraint*>(this->mChar->GetJoint(cSimMonopedHopper::eJointKnee).GetConstraintHandle().mCons);
	btKnee->setPoweredLinMotor(true);
	btKnee->setMaxLinMotorForce(max_torque_knee);
	btKnee->setTargetLinMotorVelocity(-out_tau(4)/120.0 * world_scale);

	// btHip->setTargetVelocity(out_tau(3)/120.0);
	btHip->enableAngularMotor(true, -out_tau(3)/120.0, max_torque_hip);// Must have been created with axis (0,0-1), into screen
	*/
	// hip->enableAngularMotor(true, 0.0, 5.0);
	// std::cout << "out tau: " << out_tau.transpose() << std::endl;
	// out_tau *= 0.0;
}

const cMonopedHopperController::tStateDef& cMonopedHopperController::GetCurrStateDef() const
{
	return gStateDefs[mState];
}

cMonopedHopperController::tStateParams cMonopedHopperController::GetCurrParams() const
{
	cMonopedHopperController::tStateParams params = mCurrAction.mParams.segment(
			eMiscParamMax + mState * eStateParamMax, eStateParamMax);
	return params;
}

void cMonopedHopperController::SetStateParams(const tStateParams& params)
{
	mImpPDCtrl.SetTargetTheta(cSimMonopedHopper::eJointHip, params.segment(eStateParamHip, 1));
	mImpPDCtrl.SetTargetTheta(cSimMonopedHopper::eJointKnee, params.segment(eStateParamKnee, 1));
}

void cMonopedHopperController::SetupPassiveMode()
{
	tStateParams params;
	params.setZero();
	SetStateParams(params);
}


bool cMonopedHopperController::CheckContact(cSimMonopedHopper::eJoint joint_id) const
{
	assert(joint_id != cSimMonopedHopper::eJointInvalid);
	const auto& body_part = mChar->GetBodyPart(joint_id);
	bool contact = body_part->IsInContact();
	return contact;
}

bool cMonopedHopperController::IsActiveVFEffector(cSimMonopedHopper::eJoint joint_id) const
{
	bool valid_effector = ((mState == eStateDownStance || mState == eStateUpStance)
		&& joint_id == cSimMonopedHopper::eJointKnee);

	bool active_effector = valid_effector && CheckContact(joint_id);
	return active_effector;
}

tVector cMonopedHopperController::GetEffectorVF(cSimMonopedHopper::eJoint joint_id) const
{
	tVector force = tVector::Zero();
	switch (joint_id)
	{
	// TODO fix later, should be one force for single effector
	case cSimMonopedHopper::eJointKnee:
		// force = GetBackForce();
		break;
	default:
		assert(false); // unsupported effector
		break;
	}
	return force;
}

Eigen::MatrixXd cMonopedHopperController::BuildContactBasis(const Eigen::VectorXd& pose, bool& out_has_support) const
{
	const int num_basis = 2;
	int rows = static_cast<int>(pose.size());
	const int cols = gNumEndEffectors * num_basis;

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
		cSimMonopedHopper::eJoint joint_id = gEndEffectors[e];
		bool valid_support = CheckContact(joint_id);

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

const std::string& cMonopedHopperController::GetStateName(eState state) const
{
	return gStateDefs[state].mName;
}

const std::string& cMonopedHopperController::GetStateParamName(eStateParam param) const
{
	return gStateParamNames[param];
}

void cMonopedHopperController::GetOptParams(const Eigen::VectorXd& ctrl_params, Eigen::VectorXd& out_opt_params) const
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

double cMonopedHopperController::GetGetMagicGain() const
{
	return mCurrAction.mParams[eMiscParamMagicGain];
}

/*
std::string cMonopedHopperController::BuildOptParamsJson(const Eigen::VectorXd& params) const
{
	assert(params.size() == GetNumOptParams());
	std::string json = "";
	int idx = 0;

	json += "\"" + gMiscParamsKey + "\": \n{\n";
	for (int i = 0; i < eMiscParamMax; ++i)
	{
		if (i != 0)
		{
			json += ",\n";
		}
		json += "\"" + gMiscParamsNames[i] +  "\": " + std::to_string(params[idx++]);
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
			json += "\"" + gStateParamNames[j] + "\": " + std::to_string(params[idx++]);
		}
		json += "\n}";
	}
	json += "\n}\n";

	json = "{" + json + "}";
	return json;
}
*/
void cMonopedHopperController::BuildStateParamsFromPose(const Eigen::VectorXd& pose, tStateParams& out_params)
{
	assert(false); // turn back now, there be dragons
}


const Eigen::VectorXd& cMonopedHopperController::GetCtrlParams(int ctrl_id) const
{
	return mCtrlParams[ctrl_id];
}

bool cMonopedHopperController::IsCurrActionCyclic() const
{
	const tBlendAction& action = mActions[mCurrAction.mID];
	return action.mCyclic;
}

void cMonopedHopperController::ApplyAction(int action_id)
{
	cTerrainRLCharController::ApplyAction(action_id);
}

void cMonopedHopperController::ApplyAction(const tAction& action)
{
	cTerrainRLCharController::ApplyAction(action);
	TransitionState(eStateDownStance);
}

void cMonopedHopperController::NewCycleUpdate()
{
	cTerrainRLCharController::NewCycleUpdate();
	mPrevCycleTime = mCurrCycleTime;
	mCurrCycleTime = 0;
	mPrevStumbleCount = mCurrStumbleCount;
	mCurrStumbleCount = 0;
	RecordDistTraveled();
	mPrevCOM = mChar->CalcCOM();
}

void cMonopedHopperController::BlendCtrlParams(const tBlendAction& action, Eigen::VectorXd& out_params) const
{
	const Eigen::VectorXd& param0 = GetCtrlParams(action.mParamIdx0);
	const Eigen::VectorXd& param1 = GetCtrlParams(action.mParamIdx1);
	double blend = action.mBlend;
	out_params = (1 - blend) * param0 + blend * param1;
}

void cMonopedHopperController::PostProcessParams(Eigen::VectorXd& out_params) const
{
	out_params[eMiscParamMagicGain] = std::abs(out_params[eMiscParamMagicGain]);
}

void cMonopedHopperController::PostProcessAction(tAction& out_action) const
{
	PostProcessParams(out_action.mParams);
}


bool cMonopedHopperController::IsOptParam(int param_idx) const
{
	assert(param_idx >= 0 && param_idx < gNumOptParamMasks);
	return gOptParamsMasks[param_idx];
}

tVector cMonopedHopperController::GetEndEffectorContactPos(int joint_id) const
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

void cMonopedHopperController::RecordDistTraveled()
{
	tVector com = mChar->CalcCOM();
	mPrevDistTraveled = com - mPrevCOM;
}

int cMonopedHopperController::PopCommand()
{
	int cmd = mCommands.top();
	mCommands.pop();
	return cmd;
}

bool cMonopedHopperController::HasCommands() const
{
	return !mCommands.empty();
}

void cMonopedHopperController::ClearCommands()
{
	while (!mCommands.empty())
	{
		mCommands.pop();
	}
}

void cMonopedHopperController::ProcessCommand(tAction& out_action)
{
	int a_id = PopCommand();
	BuildBaseAction(a_id, out_action);
}

void cMonopedHopperController::BuildPoliStatePose(Eigen::VectorXd& out_pose) const
{
	cTerrainRLCharController::BuildPoliStatePose(out_pose);
	double root_theta;
	tVector root_axis;
	const tVector ref_axis = tVector(0, 0, 1, 0);
	mChar->GetRootRotation(root_axis, root_theta);
	if (ref_axis.dot(root_axis) < 0)
	{
		root_theta = -root_theta;
	}
	out_pose[out_pose.size() - 1] = root_theta;
}

void cMonopedHopperController::BuildPoliStateVel(Eigen::VectorXd& out_vel) const
{
	cTerrainRLCharController::BuildPoliStateVel(out_vel);
	tVector root_omega = mChar->GetRootAngVel();
	out_vel[out_vel.size() - 1] = root_omega[2];
}

int cMonopedHopperController::GetPoliStateSize() const
{
	return cTerrainRLCharController::GetPoliStateSize();
}

int cMonopedHopperController::GetPoliStateSize(ePoliState params) const
{
	int size = 0;
	switch (params)
	{
	case ePoliStateGround:
		size = GetNumGroundSamples();
		break;
#if defined(ENBLE_MAX_COORD_POSE)
	case ePoliStatePose:
		size = 1 + mChar->GetNumBodyParts() * gPosDim - 1; // -1 for root x
		break;
	case ePoliStateVel:
		size = 1 + mChar->GetNumBodyParts() * gPosDim;
		break;
#else
	case ePoliStatePose:
		size = mChar->GetNumDof() - 1; // -1 for root x
		break;
	case ePoliStateVel:
		size = mChar->GetNumDof();
		break;
#endif
	default:
		assert(false); // unsupported poli state param
		break;
	}
	return size;
}

void cMonopedHopperController::BuildBaseAction(int action_id, tAction& out_action) const
{
	const tBlendAction& blend = mActions[action_id];
	out_action.mID = blend.mID;
	BlendCtrlParams(blend, out_action.mParams);
}
