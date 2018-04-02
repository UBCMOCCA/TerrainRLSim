/*
 * BipedController3D.cpp
 *
 *  Created on: Oct 16, 2016
 *      Author: Glen
 */

#include "BipedController3D.h"
#include <iostream>
#include <ctime>
#include "util/json/json.h"

#include "sim/SimCharacter.h"
#include "sim/RBDUtil.h"
#include "util/FileUtil.h"

const int gGroundSampleRes = 16;
const double gGroundSampleMinDist = -1;

const cBipedController3D::eStance gDefaultStance = cBipedController3D::eStanceRight;
const cBipedController3D::tStateDef gStateDefs[cBipedController3D::eStateMax] =
{
	{
		"Contact",
		true,									// mTransTime;
		false,									// mTransContact;
		cBipedController3D::eStateDown			// mNext;
	},
	{
		"Down",
		true,									// mTransTime;
		false,									// mTransFootContact;
		cBipedController3D::eStatePassing		// mNext;
	},
	{
		"Passing",
		true,									// mTransTime;
		false,									// mTransContact;
		cBipedController3D::eStateUp				// mNext;
	},
	{
		"Up",
		false,									// mTransTime;
		true,									// mTransContact;
		cBipedController3D::eStateInvalid		// mNext;
	},
};

const std::string gMiscParamsNames[cBipedController3D::eMiscParamMax] = {
	"TransTime",
	"Cv",
	"Cd",
	"ForceX",
	"ForceY"
};

const std::string gStateParamNames[cBipedController3D::eStateParamMax] = {
	"RootPitchX",
	"RootPitchY",
	"RootPitchZ",
	"RootPitchTheta",
	"StanceHipSagital",
	"StanceHipCoronal",
	"StanceKneeZ",
	"StanceAnkleX",
	"StanceAnkleY",
	"StanceAnkleZ",
	"StanceAnkleTheta",
	"SwingHipSagital",
	"SwingHipCoronal",
	"SwingKneeZ",
	"SwingAnkleX",
	"SwingAnkleY",
	"SwingAnkleZ",
	"SwingAnkleTheta"
};

const cSimBiped3D::eJoint gEndEffectors[] = {
	cSimBiped3D::eJointRightAnkle,
	cSimBiped3D::eJointLeftAnkle,
};
const int gNumEndEffectors = sizeof(gEndEffectors) / sizeof(gEndEffectors[0]);

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
	false,	//eStateParamRootPitchX,
	false,	//eStateParamRootPitchY,
	false,	//eStateParamRootPitchZ,
	true,	//eStateParamRootPitchTheta,
	true,	//eStateParamStanceHipSagital,
	true,	//eStateParamStanceHipCoronal,
	true,	//eStateParamStanceKneeZ,
	false,	//eStateParamStanceAnkleX,
	false,	//eStateParamStanceAnkleY,
	false,	//eStateParamStanceAnkleZ,
	true,	//eStateParamStanceAnkleTheta,
	true,	//eStateParamSwingHipSagital,
	true,	//eStateParamSwingHipCoronal,
	true,	//eStateParamSwingKneeZ,
	false,	//eStateParamSwingAnkleX,
	false,	//eStateParamSwingAnkleY,
	false,	//eStateParamSwingAnkleZ,
	true,	//eStateParamSwingAnkleTheta,

	//Down
	false,	//eStateParamRootPitchX,
	false,	//eStateParamRootPitchY,
	false,	//eStateParamRootPitchZ,
	true,	//eStateParamRootPitchTheta,
	true,	//eStateParamStanceHipSagital,
	true,	//eStateParamStanceHipCoronal,
	true,	//eStateParamStanceKneeZ,
	false,	//eStateParamStanceAnkleX,
	false,	//eStateParamStanceAnkleY,
	false,	//eStateParamStanceAnkleZ,
	true,	//eStateParamStanceAnkleTheta,
	true,	//eStateParamSwingHipSagital,
	true,	//eStateParamSwingHipCoronal,
	true,	//eStateParamSwingKneeZ,
	false,	//eStateParamSwingAnkleX,
	false,	//eStateParamSwingAnkleY,
	false,	//eStateParamSwingAnkleZ,
	true,	//eStateParamSwingAnkleTheta,

	//Passing
	false,	//eStateParamRootPitchX,
	false,	//eStateParamRootPitchY,
	false,	//eStateParamRootPitchZ,
	true,	//eStateParamRootPitchTheta,
	true,	//eStateParamStanceHipSagital,
	true,	//eStateParamStanceHipCoronal,
	true,	//eStateParamStanceKneeZ,
	false,	//eStateParamStanceAnkleX,
	false,	//eStateParamStanceAnkleY,
	false,	//eStateParamStanceAnkleZ,
	true,	//eStateParamStanceAnkleTheta,
	true,	//eStateParamSwingHipSagital,
	true,	//eStateParamSwingHipCoronal,
	true,	//eStateParamSwingKneeZ,
	false,	//eStateParamSwingAnkleX,
	false,	//eStateParamSwingAnkleY,
	false,	//eStateParamSwingAnkleZ,
	true,	//eStateParamSwingAnkleTheta,

	//Up
	false,	//eStateParamRootPitchX,
	false,	//eStateParamRootPitchY,
	false,	//eStateParamRootPitchZ,
	true,	//eStateParamRootPitchTheta,
	true,	//eStateParamStanceHipSagital,
	true,	//eStateParamStanceHipCoronal,
	true,	//eStateParamStanceKneeZ,
	false,	//eStateParamStanceAnkleX,
	false,	//eStateParamStanceAnkleY,
	false,	//eStateParamStanceAnkleZ,
	true,	//eStateParamStanceAnkleTheta,
	true,	//eStateParamSwingHipSagital,
	true,	//eStateParamSwingHipCoronal,
	true,	//eStateParamSwingKneeZ,
	false,	//eStateParamSwingAnkleX,
	false,	//eStateParamSwingAnkleY,
	false,	//eStateParamSwingAnkleZ,
	true,	//eStateParamSwingAnkleTheta,
};
const int gNumOptParamMasks = sizeof(gOptParamsMasks) / sizeof(gOptParamsMasks[0]);
const int gPosDim = cKinTree::gPosDim - 1;

cBipedController3D::cBipedController3D() : cTerrainRLCharController()
{
	mDefaultAction = gInvalidIdx;
	mState = eStateContact;
	mEnableGravityCompensation = true;
	mEnableVirtualForces = true;
	mViewDist = 2;
}

cBipedController3D::~cBipedController3D()
{
}

void cBipedController3D::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
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

void cBipedController3D::Reset()
{
	/*
	std::shared_ptr<cSimCharacter> mChar1 = std::shared_ptr<cSimBiped3D>(new cSimBiped3D());

	// CreateCharacter(eCharRaptor, mChar1);
	// Want to keep the same controller around but change the simChar.
	cSimCharacter::tParams char_params;
	char_params.mPos = tVector::Zero();
	char_params.mCharFile = "data/characters/biped2D.txt";
	char_params.mStateFile = "data/states/biped2D/biped_jog_state.txt";
	char_params.mPlaneCons = cWorld::ePlaneConsXY;

	bool succ = mChar1->Init(this->mChar->GetWorld(), char_params);


	Clear();

	bool succ = true;
	if (char_params.mCharFile != "")
	{
		std::ifstream f_stream(char_params.mCharFile);
		Json::Reader reader;
		Json::Value root;
		succ = reader.parse(f_stream, root);
		f_stream.close();

		if (succ)
		{
			if (root["Skeleton"].isNull())
			{
				succ = false;
			}
			else
			{
				succ = mChar1->LoadSkeleton(root["Skeleton"]);
			}
		}
	}

	if (succ)
	{
		mChar1->InitDefaultState();
	}
	*/

	/*
	if (succ)
	{
		mChar1->RegisterContacts(cWorld::eContactFlagCharacter, cWorld::eContactFlagEnvironment);
		InitCharacterPos(mChar1);
		this->

		std::shared_ptr<cCharController> ctrl;
		cTerrainRLCtrlFactory::tCtrlParams ctrl_params;
		ctrl_params.mCharCtrl = cTerrainRLCtrlFactory::eCharCtrlRaptorMACE;
		ctrl_params.mCtrlParamFile = "data/characters/raptor.txt";
		ctrl_params.mChar = mChar1;
		ctrl_params.mGravity = mGravity;
		ctrl_params.mGround = mGround;
		ctrl_params.mNetFiles.resize(2);
		ctrl_params.mModelFiles.resize(2);
		ctrl_params.mNetFiles[0] = "data/policies/raptor/nets/raptor_mace3_deploy.prototxt";
		ctrl_params.mNetFiles[1] = mCriticNetFile;
		ctrl_params.mModelFiles[0] = "data/policies/raptor/models/raptor_mace3_slopes_mixed_model.h5";
		ctrl_params.mModelFiles[1] = mCriticModelFile;

		bool succ = cTerrainRLCtrlFactory::BuildController(ctrl_params, ctrl);

		if (succ && ctrl != nullptr)
		{
			mChar1->SetController(ctrl);
		}
	}
	*/
	cTerrainRLCharController::Reset();
	ClearCommands();
	mImpPDCtrl.Reset();
	SetStance(gDefaultStance);
	mPrevCOM = mChar->CalcCOM();
}

void cBipedController3D::Clear()
{
	cTerrainRLCharController::Clear();
	mImpPDCtrl.Clear();
	mCtrlParams.clear();
	ClearCommands();

	mRBDModel.reset();
	mDefaultAction = gInvalidIdx;
}

void cBipedController3D::Update(double time_step)
{
	cTerrainRLCharController::Update(time_step);
}

void cBipedController3D::UpdateCalcTau(double time_step, Eigen::VectorXd& out_tau)
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

void cBipedController3D::UpdateApplyTau(const Eigen::VectorXd& tau)
{
	mTau = tau;
	mChar->ApplyControlForces(tau);
}

void cBipedController3D::SeCtrlStateParams(eState state, const tStateParams& params)
{
	mCurrAction.mParams.segment(eMiscParamMax + state * eStateParamMax, eStateParamMax) = params;
}

void cBipedController3D::TransitionState(int state)
{
	cTerrainRLCharController::TransitionState(state);
}

void cBipedController3D::TransitionState(int state, double phase)
{
	assert(state >= 0 && state < eStateMax);
	cTerrainRLCharController::TransitionState(state, phase);

	tStateParams params = GetCurrParams();
	SetStateParams(params);
}

void cBipedController3D::SetTransTime(double time)
{
	// set duration of each state in the walk
	mCurrAction.mParams[eMiscParamTransTime] = time;
}

int cBipedController3D::GetNumStates() const
{
	return eStateMax;
}

void cBipedController3D::SetMode(eMode mode)
{
	cTerrainRLCharController::SetMode(mode);

	if (mMode == eModePassive)
	{
		SetupPassiveMode();
	}
}

void cBipedController3D::CommandAction(int action_id)
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

void cBipedController3D::CommandRandAction()
{
	int num_actions = GetNumActions();
	int a = cMathUtil::RandInt(0, num_actions);
	CommandAction(a);
}

int cBipedController3D::GetDefaultAction() const
{
	return mDefaultAction;
}

void cBipedController3D::SetDefaultAction(int action_id)
{
	if (action_id >= 0 && action_id < GetNumActions())
	{
		mDefaultAction = action_id;
	}
}

int cBipedController3D::GetNumActions() const
{
	return static_cast<int>(mActions.size());
}

void cBipedController3D::BuildCtrlOptParams(int ctrl_params_idx, Eigen::VectorXd& out_params) const
{
	GetOptParams(mCtrlParams[ctrl_params_idx], out_params);
}

void cBipedController3D::SetCtrlParams(int ctrl_params_id, const Eigen::VectorXd& params)
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

void cBipedController3D::SetCtrlOptParams(int ctrl_params_id, const Eigen::VectorXd& opt_params)
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

void cBipedController3D::BuildActionOptParams(int action_id, Eigen::VectorXd& out_params) const
{
	assert(action_id >= 0 && action_id < GetNumActions());
	const tBlendAction& action = mActions[action_id];
	Eigen::VectorXd params;
	BlendCtrlParams(action, params);
	GetOptParams(params, out_params);
}

int cBipedController3D::GetNumParams() const
{
	int num_params = eMiscParamMax + eStateMax * eStateParamMax;
	return num_params;
}

int cBipedController3D::GetNumOptParams() const
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

void cBipedController3D::FetchOptParamScale(Eigen::VectorXd& out_scale) const
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
			{
				param_buffer[idx] = angle_scale;
			}
			++idx;
		}
	}

	assert(idx == num_params);
	GetOptParams(param_buffer, out_scale);
}

void cBipedController3D::OutputOptParams(const std::string& file, const Eigen::VectorXd& params) const
{
	cController::OutputOptParams(file, params);
}

void cBipedController3D::OutputOptParams(FILE* f, const Eigen::VectorXd& params) const
{
	std::string opt_param_json = BuildOptParamsJson(params);
	fprintf(f, "%s\n", opt_param_json.c_str());
}

void cBipedController3D::ReadParams(std::ifstream& f_stream)
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

void cBipedController3D::ReadParams(const std::string& file)
{
	cTerrainRLCharController::ReadParams(file);
}

void cBipedController3D::SetParams(const Eigen::VectorXd& params)
{
	assert(params.size() == GetNumParams());
	mCurrAction.mParams = params;
	PostProcessParams(mCurrAction.mParams);
}

void cBipedController3D::BuildOptParams(Eigen::VectorXd& out_params) const
{
	GetOptParams(mCurrAction.mParams, out_params);
}

void cBipedController3D::SetOptParams(const Eigen::VectorXd& opt_params)
{
	SetOptParams(opt_params, mCurrAction.mParams);
}

void cBipedController3D::SetOptParams(const Eigen::VectorXd& opt_params, Eigen::VectorXd& out_params) const
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

void cBipedController3D::BuildFromMotion(int ctrl_params_idx, const cMotion& motion)
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

double cBipedController3D::CalcReward() const
{
	tVector target_vel = GetTargetVel();

	const double vel_reward_w = 0.7;
	double vel_reward = 0;
	const double stumble_reward_w = 0.3;
	double stumble_reward = 0;
	const double root_pitch_w = 0.12;
	double root_pitch = 0;

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

		root_pitch = std::fabs(GetRootPitch());

		if (avg_vel < 0)
		{
			vel_reward = 0;
			stumble_reward = 0;
		}
	}

	double reward = 0;
	reward += (vel_reward_w * vel_reward)
			+ (stumble_reward_w * stumble_reward)
			+ (root_pitch_w * root_pitch);

	return reward;
}

tVector cBipedController3D::GetTargetVel() const
{
	return tVector(2.0, 0, 0, 0);
	// return tVector(1.5, 0, 0, 0);
}

cBipedController3D::eStance cBipedController3D::GetStance() const
{
	return mStance;
}

double cBipedController3D::GetPrevCycleTime() const
{
	return mPrevCycleTime;
}

const tVector& cBipedController3D::GetPrevDistTraveled() const
{
	return mPrevDistTraveled;
}

void cBipedController3D::BuildNormPose(Eigen::VectorXd& pose) const
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

bool cBipedController3D::LoadControllers(const std::string& file)
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

std::string cBipedController3D::BuildOptParamsJson(const Eigen::VectorXd& opt_params) const
{
	assert(opt_params.size() == GetNumOptParams());
	Eigen::VectorXd param_buffer = mCurrAction.mParams;
	SetOptParams(opt_params, param_buffer);
	std::string json = BuildParamsJson(param_buffer);
	return json;
}

std::string cBipedController3D::BuildParamsJson(const Eigen::VectorXd& params) const
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

void cBipedController3D::DebugPrintAction(const tAction& action) const
{
	printf("Action ID: %i\n", action.mID);
	std::string opt_param_json = BuildParamsJson(action.mParams);
	printf("Action params: \n %s\n", opt_param_json.c_str());
}

void cBipedController3D::ResetParams()
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

bool cBipedController3D::ParseControllers(const Json::Value& root)
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

bool cBipedController3D::ParseControllerFiles(const Json::Value& root)
{
	bool succ = true;
	std::vector<std::string> files;
	assert(root.isArray());

	int num_files = root.size();
	files.resize(num_files);
	for (int f = 0; f < num_files; ++f)
	{
		std::string curr_file = root.get(f, 0).asString();
		ReadParams(curr_file);
	}
	return succ;
}

bool cBipedController3D::ParseActions(const Json::Value& root)
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

bool cBipedController3D::ParseAction(const Json::Value& root, tBlendAction& out_action) const
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

bool cBipedController3D::HasCtrlParams() const
{
	return mCtrlParams.size() > 0;
}

void cBipedController3D::UpdateState(double time_step)
{
	const cBipedController3D::tStateDef& state = GetCurrStateDef();
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

void cBipedController3D::UpdateAction()
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

void cBipedController3D::UpdateRBDModel()
{
	const Eigen::VectorXd& pose = mChar->GetPose();
	const Eigen::VectorXd& vel = mChar->GetVel();

	mRBDModel->Update(pose, vel);
	cRBDUtil::BuildJacobian(*mRBDModel.get(), mJacobian);
}

void cBipedController3D::UpdatePDCtrls(double time_step, Eigen::VectorXd& out_tau)
{
	mImpPDCtrl.UpdateControlForce(time_step, out_tau);
}

void cBipedController3D::UpdateStumbleCounter(double time_step)
{
	bool stumbled = mChar->HasStumbled();
	if (stumbled)
	{
		mCurrStumbleCount += time_step;
	}
}

void cBipedController3D::UpdateStanceHip()
{
	int stance_hip = GetStanceHip();
	int stance_toe = GetStanceAnkle();
	bool active = IsActiveVFEffector(stance_toe);
	mImpPDCtrl.GetPDCtrl(stance_hip).SetActive(!active);
}

#define G_EPSILON 0.0000001f
#define IS_ZERO(value) (value < G_EPSILON &&  value > -G_EPSILON)
#define SGN(x) (((x)<0)?(-1):(1))
/**
	This method is used to compute the PD torque that aligns a child coordinate frame to a parent coordinate frame.
	Given: the current relative orientation of two coordinate frames (child and parent), the relative angular velocity,
	the desired values for the relative orientation and ang. vel, as well as the virtual motor's PD gains. The torque
	returned is expressed in the coordinate frame of the 'parent'.
*/
tVector cBipedController3D::computePDTorque(const tQuaternion& qRel, const tQuaternion& qRelD, const tVector& wRel, const tVector& wRelD, const cPDController& params) const
{
	tVector torque;
	//the torque will have the form:
	// T = kp*D(qRelD, qRel) + kd * (wRelD - wRel)

	//Note: There can be problems computing the proper torque from the quaternion part, because q and -q 
	//represent the same orientation. To make sure that we get the correct answer, we'll take into account
	//the sign of the scalar part of qErr - both this and the v part will change signs in the same way if either 
	//or both of qRel and qRelD are negative
	//	Quaternion qErr = qRel.getComplexConjugate() * qRelD;
	tQuaternion qErr = qRel.conjugate();
	qErr *= qRelD;

	tVector qErrV(qErr.x(), qErr.y(), qErr.z(), 0.0);
	//qErr.v also contains information regarding the axis of rotation and the angle (sin(theta)), but I want to scale it by theta instead
	double sinTheta = qErrV.norm();
	if (sinTheta > 1)
	{
		sinTheta = 1;
	}

	if (IS_ZERO(sinTheta)) {
		//avoid the divide by close-to-zero. The orientations match, so the proportional component of the torque should be 0
	}
	else {
		double absAngle = 2 * asin(sinTheta);
		torque = qErrV;
		torque *= 1 / sinTheta * absAngle * (-params.GetKp()) * SGN(qErr.w());
		//		torque = qErr.v/sinTheta * absAngle * (-cParams->kp) * SGN(qErr.s);
	}

	//qErr represents the rotation from the desired child frame to the actual child frame, which
	//means that the torque is now expressed in child coordinates. We need to express it in parent coordinates!
	/// torque = qRel.rotate(torque);
	torque = cMathUtil::QuatRotVec(qRel, torque);
	//the angular velocities are stored in parent coordinates, so it is ok to add this term now
	torque += (wRelD - wRel) * (-params.GetKd());
	/// torque *= cParams->strength;

	//now the torque is stored in parent coordinates - we need to scale it and apply torque limits
	/// scaleAndLimitTorque(&torque, cParams, qRel.getComplexConjugate());

	//and we're done...
	return torque;
}

void cBipedController3D::ApplySwingFeedback(Eigen::VectorXd& out_tau)
{

	// Find the sagital plane wrt root
	tQuaternion rootRot = mChar->GetRootRotation();
	// cJoint swingLegRot = mChar->GetJoint(this->GetSwingHip());

	Eigen::Vector3d ea = rootRot.toRotationMatrix().eulerAngles(0, 1, 2);

	Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd yawAngle(-ea(1), Eigen::Vector3d::UnitY()); // want to reverse this rotation to get back to world coords
	Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitZ());

	Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

	Eigen::Matrix3d rotationMatrix = q.matrix();

	double cv = GetCv();
	double cd = GetCd();

	bool first_half = mState == eStateContact || mState == eStateDown;
	cd = (first_half) ? 0 : cd;
	cv = (first_half) ? cv : 0;

	tVector com = mChar->CalcCOM();
	tVector com_vel = mChar->CalcCOMVel();

	int stance_toe_id = GetStanceAnkle();
	int swing_hip_id = GetSwingHip();
	int stance_hip_id = GetStanceHip();

	// This works if root is aligned with x0axis
	tVector toe_pos = mChar->GetBodyPart(stance_toe_id)->GetPos();
	tVector d = com - toe_pos;
	// rotate back into world coordinates. Wrt y-axis only?
	Eigen::Vector3d world_d = rotationMatrix * Eigen::Vector3d (d[0], d[1], d[2]);
	Eigen::Vector3d world_com_vel = rotationMatrix * Eigen::Vector3d(com_vel[0], com_vel[1], com_vel[2]);
	double d_theta_x = cd * -world_d[2] + cv * -world_com_vel[2]; // depends on z distance // Can't figure out why this should be inverted..
	double d_theta_z = cd * world_d[0] + cv * world_com_vel[0]; // depends on x distance

	tStateParams params = GetCurrParams();
	double theta0_z = params(eStateParamSwingHipSagital);
	double theta0_x = params(eStateParamSwingHipCoronal);
	/*
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();

	Eigen::VectorXd root_rot_ = params.segment(eStateParamSwingHipW, 4);
	tQuaternion hip_rot(root_rot_(0), root_rot_(1), root_rot_(2), root_rot_(3));
	tVector root_axis;
	double root_theta;
	cMathUtil::QuaternionToAxisAngle(hip_rot, root_axis, root_theta);
	*/
	double theta_new_x = theta0_x + d_theta_x;
	double theta_new_z = theta0_z + d_theta_z;

	// Need to convert this back to a target quaternion now.
	// And back into joint relative coordinates?
	Eigen::AngleAxisd rollAngle2(theta_new_x, Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd yawAngle2(0.0, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle2(theta_new_z, Eigen::Vector3d::UnitZ());

	Eigen::Quaternion<double> q2 = rollAngle2 * yawAngle2 * pitchAngle2;

	Eigen::VectorXd theta_vec(4);
	theta_vec[0] = q2.w();
	theta_vec[1] = q2.x();
	theta_vec[2] = q2.y();
	theta_vec[3] = q2.z();
	mImpPDCtrl.SetTargetTheta(swing_hip_id, theta_vec);
	
}

void cBipedController3D::ApplyStanceFeedback(Eigen::VectorXd& out_tau)
{
	
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();

	
	
	int stance_toe = GetStanceAnkle();
	
	int stance_hip_id = GetStanceHip();
	int swing_hip_id = GetSwingHip();

	const cJoint& stance_leg = mChar->GetJoint(this->GetSwingHip());
	tMatrix rot_mat_test = stance_leg.BuildWorldTrans();
	tQuaternion stance_leg_rot = cMathUtil::RotMatToQuaternion(rot_mat_test);

	int stance_hip_offset = cKinTree::GetParamOffset(joint_mat, stance_hip_id);
	int stance_hip_size = cKinTree::GetParamSize(joint_mat, stance_hip_id);
	Eigen::VectorXd hip_tau = Eigen::VectorXd::Ones(stance_hip_size) * 0.0;

	const int root_joints[] =
	{
		cSimBiped3D::eJointRightHip,
		cSimBiped3D::eJointLeftHip,
		//cSimBiped3D::eJointSpine0,
		//cSimBiped3D::eJointTail0
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
				int swing_hip_size = cKinTree::GetParamSize(joint_mat, swing_hip_id);
				auto swing_tau = out_tau.segment(param_offset, swing_hip_size);
				std::cout << "Swing hip tau: " << swing_tau.transpose() << std::endl;
				hip_tau += -swing_tau;
			}
		}

		tQuaternion target_root_rot = GetTargetRootPitch3D();
		tQuaternion root_pitch = GetRootRot3D();
		tVector root_omega = mChar->GetRootAngVel();

		const cPDController& stance_pd = mImpPDCtrl.GetPDCtrl(stance_hip_id);
		const double kp = stance_pd.GetKp();
		const double kd = stance_pd.GetKd();

		tQuaternion root_q_diff = root_pitch.conjugate() * target_root_rot;
		tVector axis;
		double theta;
		cMathUtil::QuaternionToAxisAngle(root_q_diff, axis, theta);
		tVector root_torque_ = (kp * theta) * axis; // Does some fancy exponential map thing??
		// tVector root_torque_(ea[0], ea[1], ea[2], 0);
		// tVector root_tau = kp * (root_torque_) + kd * (-root_omega);
		tVector root_tau = computePDTorque(root_pitch, target_root_rot, root_omega, tVector(0, 0, 0, 0), stance_pd);
		// Need to rotate this torque into swing leg coordinates not root coordinates.
		tMatrix root_to_stance;
		// cKinTree::BodyJointTrans(root_to_stance, stance_hip_id);
		// rotate torque into child coordinates
		// tQuaternion root_to_stance_rot = (stance_leg_rot * root_pitch.conjugate());
		// root_tau = cMathUtil::QuatRotVec(root_to_stance_rot, root_tau);
		std::cout << "root tau: " << root_tau.transpose() << std::endl;
		hip_tau += (-root_tau * 1.0); // scaling

		auto stance_tau = out_tau.segment(stance_hip_offset, stance_hip_size);
		// std::cout << "Stance Hip tau before: " << stance_tau.transpose() << std::endl;
		stance_tau += hip_tau;
		// std::cout << "Stance Hip tau before: " << stance_tau.transpose() << std::endl;
	}
	
}

void cBipedController3D::ApplyGravityCompensation(Eigen::VectorXd& out_tau)
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

void cBipedController3D::ApplyVirtualForces(Eigen::VectorXd& out_tau)
{
	for (int e = 0; e < gNumEndEffectors; ++e)
	{
		cSimBiped3D::eJoint joint_id = gEndEffectors[e];
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

const cBipedController3D::tStateDef& cBipedController3D::GetCurrStateDef() const
{
	return gStateDefs[mState];
}

cBipedController3D::tStateParams cBipedController3D::GetCurrParams() const
{
	tStateParams params = mCurrAction.mParams.segment(eMiscParamMax + mState * eStateParamMax, eStateParamMax);
	return params;
}

/*
double cBipedController3D::GetTargetRootPitch() const
{
	tStateParams params = GetCurrParams();
	double pitch = params(eStateParamRootPitch);
	return pitch;
}
*/

tQuaternion cBipedController3D::GetTargetRootPitch3D() const
{
	tStateParams params = GetCurrParams();
	Eigen::VectorXd pitch = params.segment(eStateParamRootPitchX, 4);
	tQuaternion rootRotT = cMathUtil::AxisAngleToQuaternion(tVector(pitch(0), pitch(1), pitch(2), 0), pitch(3));
	return rootRotT;
}


tQuaternion cBipedController3D::GetRootRot3D() const
{
	tQuaternion rootRot = mChar->GetRootRotation();
	return rootRot;
}


double cBipedController3D::GetRootPitch() const
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


void cBipedController3D::SetStateParams(const tStateParams& params)
{
#if defined(ENABLE_SPINE_CURVE)
	double spine_theta = params(eStateParamSpineCurve);
	for (int i = 0; i < gNumSpineJoints; ++i)
	{
		cSimBiped3D::eJoint joint = gSpineJoints[i];
		mImpPDCtrl.SetTargetTheta(joint, spine_theta);
	}
#endif
	/// Explore this more in the future maybe I should init from the feedback rules here...
	// mImpPDCtrl.SetTargetTheta(cSimBiped3D::eJointRoot, params.segment(eStateParamRootPitchW, 4));
	// mImpPDCtrl.SetTargetTheta(GetStanceHip(), params.segment(eStateParamStanceHipSagital, 2)); I think this is a bad idea anyway
	mImpPDCtrl.SetTargetTheta(GetStanceKnee(), params.segment(eStateParamStanceKneeZ, 1));
	Eigen::VectorXd parm = params.segment(eStateParamStanceAnkleX, 3);
	parm.normalize();
	tQuaternion rot_ = cMathUtil::AxisAngleToQuaternion(tVector(parm(0), parm(1), parm(2), 0.0), params(eStateParamStanceAnkleTheta));
	parm.resize(4);
	parm(0) = rot_.w();
	parm(1) = rot_.x();
	parm(2) = rot_.y();
	parm(3) = rot_.z();
	mImpPDCtrl.SetTargetTheta(GetStanceAnkle(), parm);
	// mImpPDCtrl.SetTargetTheta(GetSwingHip(), params.segment(eStateParamSwingHipSagital, 2)); Should be set from feedback
	mImpPDCtrl.SetTargetTheta(GetSwingKnee(), params.segment(eStateParamSwingKneeZ, 1));
	Eigen::VectorXd parm2 = params.segment(eStateParamSwingAnkleX, 3);
	parm2.normalize();
	tQuaternion rot__ = cMathUtil::AxisAngleToQuaternion(tVector(parm2(0), parm2(1), parm2(2), 0.0), params(eStateParamSwingAnkleTheta));
	parm2.resize(4);
	parm2(0) = rot__.w();
	parm2(1) = rot__.x();
	parm2(2) = rot__.y();
	parm2(3) = rot__.z();
	mImpPDCtrl.SetTargetTheta(GetSwingAnkle(), parm2);
}

void cBipedController3D::SetupPassiveMode()
{
	tStateParams params;
	params.setZero();
	SetStateParams(params);
}

double cBipedController3D::GetTransTime() const
{
	return mCurrAction.mParams[eMiscParamTransTime];
}

double cBipedController3D::GetCv() const
{
	return mCurrAction.mParams[eMiscParamCv];
}

double cBipedController3D::GetCd() const
{
	return mCurrAction.mParams[eMiscParamCd];
}

bool cBipedController3D::CheckContact(int joint_id) const
{
	assert(joint_id != cSimBiped3D::eJointInvalid);
	const auto& body_part = mChar->GetBodyPart(joint_id);
	bool contact = body_part->IsInContact();
	return contact;
}

bool cBipedController3D::IsActiveVFEffector(int joint_id) const
{
	int state = GetState();
	bool valid_effector = (joint_id == GetStanceAnkle()) && (state == eStateContact || state == eStateDown);
	bool active_effector = valid_effector && CheckContact(joint_id);
	return active_effector;
}

tVector cBipedController3D::GetEffectorVF() const
{
	return tVector(mCurrAction.mParams[eMiscParamForceX], mCurrAction.mParams[eMiscParamForceY], 0, 0);
}

Eigen::MatrixXd cBipedController3D::BuildContactBasis(const Eigen::VectorXd& pose, bool& out_has_support) const
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
		cSimBiped3D::eJoint joint_id = gEndEffectors[e];
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

const std::string& cBipedController3D::GetStateName(eState state) const
{
	return gStateDefs[state].mName;
}

const std::string& cBipedController3D::GetStateParamName(eStateParam param) const
{
	return gStateParamNames[param];
}

void cBipedController3D::GetOptParams(const Eigen::VectorXd& ctrl_params, Eigen::VectorXd& out_opt_params) const
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

void cBipedController3D::BuildStateParamsFromPose(const Eigen::VectorXd& pose, tStateParams& out_params)
{
	const cSimBiped3D::eJoint ctrl_joints[] =
	{	// the sagital and coronal state params are difficult to compute here.
		// cSimBiped3D::eJointRightHip,
		cSimBiped3D::eJointRightKnee,
		cSimBiped3D::eJointRightAnkle,
		// cSimBiped3D::eJointLeftHip,
		cSimBiped3D::eJointLeftKnee,
		cSimBiped3D::eJointLeftAnkle,
	};
	const int num_ctrl_joints = sizeof(ctrl_joints) / sizeof(ctrl_joints[0]);

	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();

	tQuaternion root_rot = cKinTree::GetRootRot(joint_mat, pose);
	out_params(eStateParamRootPitchTheta) = root_rot.w(); // hack to compile does not make sense..
	out_params(eStateParamRootPitchX) = root_rot.x();
	out_params(eStateParamRootPitchY) = root_rot.y();
	out_params(eStateParamRootPitchZ) = root_rot.z();
	// out_params(eStateParamSpineCurve) = CalcSpineCurve(pose);
	size_t j = 0;
	for (int i = 0; i < num_ctrl_joints; ++i)
	{
		tVector axis;
		double tar_theta = 0;
		int joint_id = ctrl_joints[i];
		cKinTree::eJointType joint_type = mImpPDCtrl.GetPDCtrl(joint_id).GetJoint().GetType();
		if (joint_type == cKinTree::eJointTypeRevolute)
		{
			j = j + 2; // hack because of different lengths of state parameters
			if (mImpPDCtrl.UseWorldCoord(joint_id))
			{
				cKinTree::CalcJointWorldTheta(joint_mat, pose, joint_id, axis, tar_theta);
			}
			else
			{
				Eigen::VectorXd joint_params;
				cKinTree::GetJointParams(joint_mat, pose, joint_id, joint_params);
				tar_theta = joint_params[0];
			}

			tVector ref_axis = tVector(0, 0, 1, 0);
			if (ref_axis.dot(axis) < 0)
			{
				tar_theta = -tar_theta;
			}
			out_params(eStateParamStanceHipSagital + j) = tar_theta;
			j = j + 1;
		}
		else if (joint_type == cKinTree::eJointTypeSpherical)
		{
			if (mImpPDCtrl.UseWorldCoord(joint_id))
			{
				cKinTree::CalcJointWorldTheta(joint_mat, pose, joint_id, axis, tar_theta);
				tQuaternion target_Rot(Eigen::AngleAxisd(tar_theta, Eigen::Vector3d(axis(0), axis(1), axis(2))));
				out_params(eStateParamStanceHipSagital + j) -= target_Rot.w();
				out_params(eStateParamStanceHipSagital + j + 1) -= target_Rot.x();
				out_params(eStateParamStanceHipSagital + j + 2) -= target_Rot.y();
				out_params(eStateParamStanceHipSagital + j + 3) -= target_Rot.z();
			}
			else
			{
				Eigen::VectorXd joint_params;
				cKinTree::GetJointParams(joint_mat, pose, joint_id, joint_params);
				out_params(eStateParamStanceHipSagital + j) -= joint_params[0];
				out_params(eStateParamStanceHipSagital + j + 1) -= joint_params[1];
				out_params(eStateParamStanceHipSagital + j + 2) -= joint_params[2];
				out_params(eStateParamStanceHipSagital + j + 3) -= joint_params[3];
			}
			j = j + 4;
		}
		else
		{
			std::cout << "Only revolute and sphereical joints support pose calculation" << std::endl;
			assert(false);
		}
	}
}

const Eigen::VectorXd& cBipedController3D::GetCtrlParams(int ctrl_id) const
{
	return mCtrlParams[ctrl_id];
}

bool cBipedController3D::IsCurrActionCyclic() const
{
	const tBlendAction& action = mActions[mCurrAction.mID];
	return action.mCyclic;
}

void cBipedController3D::ApplyAction(int action_id)
{
	cTerrainRLCharController::ApplyAction(action_id);
}

void cBipedController3D::ApplyAction(const tAction& action)
{
	cTerrainRLCharController::ApplyAction(action);
	TransitionState(eStateContact);
}

void cBipedController3D::NewCycleUpdate()
{
	cTerrainRLCharController::NewCycleUpdate();
	mPrevCycleTime = mCurrCycleTime;
	mCurrCycleTime = 0;
	mPrevStumbleCount = mCurrStumbleCount;
	mCurrStumbleCount = 0;
	RecordDistTraveled();
	mPrevCOM = mChar->CalcCOM();
}

void cBipedController3D::BlendCtrlParams(const tBlendAction& action, Eigen::VectorXd& out_params) const
{
	const Eigen::VectorXd& param0 = GetCtrlParams(action.mParamIdx0);
	const Eigen::VectorXd& param1 = GetCtrlParams(action.mParamIdx1);
	double blend = action.mBlend;
	out_params = param0 + (blend * (param1- param0));

	// diff = p0.inverse() * p1
	// 
	// p' = p0 * power(diff, blend) // equivalent to p' <- p0 * (diff)^blend
}

void cBipedController3D::PostProcessParams(Eigen::VectorXd& out_params) const
{
	out_params[eMiscParamTransTime] = std::abs(out_params[eMiscParamTransTime]);
	out_params[eMiscParamCv] = std::abs(out_params[eMiscParamCv]);
	out_params[eMiscParamCd] = std::abs(out_params[eMiscParamCd]);
}

void cBipedController3D::PostProcessAction(tAction& out_action) const
{
	PostProcessParams(out_action.mParams);
}

bool cBipedController3D::IsOptParam(int param_idx) const
{
	assert(param_idx >= 0 && param_idx < gNumOptParamMasks);
	return gOptParamsMasks[param_idx];
}

void cBipedController3D::BuildPoliStatePose(Eigen::VectorXd& out_pose) const
{
	cTerrainRLCharController::BuildPoliStatePose(out_pose);
	eStance stance = GetStance();
	if (stance != gDefaultStance)
	{
		FlipPoliPoseStance(out_pose);
	}
}

void cBipedController3D::BuildPoliStateVel(Eigen::VectorXd& out_vel) const
{
	cTerrainRLCharController::BuildPoliStateVel(out_vel);
	eStance stance = GetStance();
	if (stance != gDefaultStance)
	{
		FlipPoliPoseStance(out_vel);
	}
}

void cBipedController3D::FlipStance()
{
	SetStance((mStance == eStanceRight) ? eStanceLeft : eStanceRight);
}

void cBipedController3D::SetStance(eStance stance)
{
	mStance = stance;
	mImpPDCtrl.GetPDCtrl(GetStanceHip()).SetActive(false);
	mImpPDCtrl.GetPDCtrl(GetSwingHip()).SetActive(true);

	TransitionState(GetState(), GetPhase());
}

void cBipedController3D::FlipPoseStance(Eigen::VectorXd& out_pose) const
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();

	int num_dofs = mChar->GetNumDof();
	int last_body_joint = cSimBiped3D::eJointLeftAnkle;
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

void cBipedController3D::FlipPoliPoseStance(Eigen::VectorXd& out_pose) const
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

int cBipedController3D::GetNumJointsPerLeg() const
{
	int hip_id = GetStanceHip();
	int toe_id = GetStanceAnkle();
	int num_joints = std::abs(toe_id - hip_id) + 1;
	return num_joints;
}

int cBipedController3D::GetSwingHip() const
{
	return (GetStance() == eStanceRight) ? cSimBiped3D::eJointLeftHip : cSimBiped3D::eJointRightHip;
}

int cBipedController3D::GetSwingKnee() const
{
	return (GetStance() == eStanceRight) ? cSimBiped3D::eJointLeftKnee : cSimBiped3D::eJointRightKnee;
}

int cBipedController3D::GetSwingAnkle() const
{
	return (GetStance() == eStanceRight) ? cSimBiped3D::eJointLeftAnkle : cSimBiped3D::eJointRightAnkle;
}

int cBipedController3D::GetStanceHip() const
{
	return (GetStance() == eStanceRight) ? cSimBiped3D::eJointRightHip : cSimBiped3D::eJointLeftHip;
}

int cBipedController3D::GetStanceKnee() const
{
	return (GetStance() == eStanceRight) ? cSimBiped3D::eJointRightKnee : cSimBiped3D::eJointLeftKnee;
}

int cBipedController3D::GetStanceAnkle() const
{
	return (GetStance() == eStanceRight) ? cSimBiped3D::eJointRightAnkle : cSimBiped3D::eJointLeftAnkle;
}

tVector cBipedController3D::GetEndEffectorContactPos(int joint_id) const
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

void cBipedController3D::RecordDistTraveled()
{
	tVector com = mChar->CalcCOM();
	mPrevDistTraveled = com - mPrevCOM;
}

int cBipedController3D::PopCommand()
{
	int cmd = mCommands.top();
	mCommands.pop();
	return cmd;
}

bool cBipedController3D::HasCommands() const
{
	return !mCommands.empty();
}

void cBipedController3D::ClearCommands()
{
	while (!mCommands.empty())
	{
		mCommands.pop();
	}
}

void cBipedController3D::ProcessCommand(tAction& out_action)
{
	int a_id = PopCommand();
	BuildBaseAction(a_id, out_action);
}

void cBipedController3D::BuildBaseAction(int action_id, tAction& out_action) const
{
	const tBlendAction& blend = mActions[action_id];
	out_action.mID = blend.mID;
	BlendCtrlParams(blend, out_action.mParams);
}


int cBipedController3D::GetNumGroundSamples() const
{
	return gGroundSampleRes * gGroundSampleRes;
}

tVector cBipedController3D::CalcGroundSamplePos(int s) const
{
	tVector origin = CalcGroundSampleOrigin();
	tVector size = CalcGroundSampleSize();

	const int num_samples = GetNumGroundSamples();
	double u = static_cast<double>(s % gGroundSampleRes) / (gGroundSampleRes - 1);
	double v = static_cast<double>(s / gGroundSampleRes) / (gGroundSampleRes - 1);

	double x = (u - 0.5) * size[0];
	double z = (v - 0.5) * size[2];

	tVector sample_pos = origin + tVector(x, 0, z, 0);
	sample_pos[3] = 1;
	sample_pos = mGroundSampleTrans * sample_pos;
	sample_pos[3] = 0;
	return sample_pos;
}

tVector cBipedController3D::CalcGroundSampleOrigin() const
{
	double max_dist = GetViewDist();
	double min_dist = gGroundSampleMinDist;
	tVector origin = tVector(0.5 * (max_dist + min_dist), 0, 0, 1);
	return origin;
}

tVector cBipedController3D::CalcGroundSampleSize() const
{
	double max_dist = GetViewDist();
	double min_dist = gGroundSampleMinDist;
	double w = max_dist - min_dist;
	return tVector(w, 0, w, 0);
}
