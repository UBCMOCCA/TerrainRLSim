#include "ScenarioExpMimic.h"
#include "sim/CtController.h"
#include "sim/MixController.h"
#include "sim/CtTrackController.h"

const int gNumWarmupCycles = 0;

#define ENABLE_COACH_TAU_AVG

cScenarioExpMimic::cScenarioExpMimic()
{
	mMimicMode = eMimicModeTau;
	mCoachActionCount = 0;
	mCharCtrlID = gInvalidIdx;
	mCoachCtrlID = gInvalidIdx;
	mCoachBlend = 0;
	mEnableCoachActiveProb = false;
	mCoachWarmupMax = 0;
	mCoachWarmupTimer = 0;
	mCouchWarmupTimeMax = 0;
}

cScenarioExpMimic::~cScenarioExpMimic()
{
}

void cScenarioExpMimic::Reset()
{
	cScenarioExp::Reset();
	mKinChar->Reset();
	ResetCoachWarmupCounter();
	mCoachActionCount = 0;

	if (IsCoachWarmup())
	{
		ApplyCoachBlend(1);
	}
}

void cScenarioExpMimic::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioExp::ParseArgs(parser);

	std::string mode_str = "";
	parser->ParseString("mimic_mode", mode_str);
	ParseMimicMode(mode_str, mMimicMode);

	parser->ParseString("motion_file", mMotionFile);
	std::string fpath;
	bool succ = parser->ParseString("relative_file_path", fpath);
	if (succ)
	{
		mMotionFile = fpath + mMotionFile;
	}

	mCoachCtrlParams = mCtrlParams;

	std::string coach_ctrl_str = "";
	parser->ParseString("coach_ctrl", coach_ctrl_str);
	cTerrainRLCtrlFactory::ParseCharCtrl(coach_ctrl_str, mCoachCtrlParams.mCharCtrl);
	parser->ParseDouble("coach_ctrl_ct_query_rate", mCoachCtrlParams.mCtQueryRate);

	parser->ParseString("coach_policy_net", mCoachPoliNetFile);
	parser->ParseString("coach_policy_model", mCoachPoliModelFile);
	parser->ParseString("coach_critic_net", mCoachCriticNetFile);
	parser->ParseString("coach_critic_model", mCoachCriticModelFile);

	parser->ParseDouble("coach_warmup_max", mCoachWarmupMax);
}

void cScenarioExpMimic::Init()
{
	bool succ = BuildKinCharacter(mKinChar);
	if (!succ)
	{
		printf("Failed to build kin character\n");
		assert(false);
	}

	cScenarioExp::Init();
	EnableCoachCtrl(false);
	ResetCoachWarmupCounter();

	mCoachActionCount = 0;
	mCoachAction = Eigen::VectorXd::Zero(mChar->GetNumDof());
	mCoachActionSample = Eigen::VectorXd::Zero(mChar->GetNumDof());
}

const std::shared_ptr<cKinCharacter>& cScenarioExpMimic::GetKinChar() const
{
	return mKinChar;
}

void cScenarioExpMimic::EnableCoachCtrl(bool enable)
{
	if (enable)
	{
		SetCoachBlend(1);
	}
	else
	{
		SetCoachBlend(0);
	}
}

void cScenarioExpMimic::SetCoachBlend(double blend)
{
	mCoachBlend = blend;
	ApplyCoachBlend(blend);
}

void cScenarioExpMimic::EnableCoachActiveProb(bool enable)
{
	mEnableCoachActiveProb = enable;
}

void cScenarioExpMimic::SetCoachActiveProb(double prob)
{
	mCoachActiveProb = prob;
}

int cScenarioExpMimic::GetCharCtrlID() const
{
	return mCharCtrlID;
}

int cScenarioExpMimic::GetCoachCtrlID() const
{
	return mCoachCtrlID;
}

std::string cScenarioExpMimic::GetName() const
{
	return "Mimic Exploration";
}

void cScenarioExpMimic::ParseMimicMode(const std::string& str, eMimicMode& out_mode) const
{
	if (str == "")
	{
	}
	else if (str == "tau")
	{
		out_mode = eMimicModeTau;
	}
	else if (str == "action")
	{
		out_mode = eMimicModeAction;
	}
	else
	{
		assert(false); // unsupported mimic mode
	}
}

bool cScenarioExpMimic::BuildKinCharacter(std::shared_ptr<cKinCharacter>& out_char) const
{
	auto kin_char = std::shared_ptr<cKinCharacter>(new cKinCharacter());
	kin_char->EnableVelUpdate(true);
	bool succ = kin_char->Init(mCharParams.mCharFile, mMotionFile);
	if (succ)
	{
		out_char = kin_char;
	}
	return succ;
}

bool cScenarioExpMimic::BuildController(std::shared_ptr<cCharController>& out_ctrl)
{
	std::shared_ptr<cTerrainRLCharController> char_ctrl;
	std::shared_ptr<cTerrainRLCharController> coach_ctrl;
	SetupCoachControllerParams(mCoachCtrlParams);
	bool succ = BuildCoachController(mCoachCtrlParams, coach_ctrl);

	{
		std::shared_ptr<cCharController> ctrl;
		succ &= cScenarioExp::BuildController(ctrl);
		char_ctrl = std::static_pointer_cast<cTerrainRLCharController>(ctrl);
	}

	auto mix_ctrl = std::shared_ptr<cMixController>(new cMixController());
	mix_ctrl->Init(mChar.get());

	mCharCtrlID = mix_ctrl->AddController(char_ctrl);
	mCoachCtrlID = mix_ctrl->AddController(coach_ctrl);
	assert(mCharCtrlID != gInvalidIdx);
	assert(mCoachCtrlID != gInvalidIdx);

	out_ctrl = mix_ctrl;
	return succ;
}

void cScenarioExpMimic::SetupCoachControllerParams(cTerrainRLCtrlFactory::tCtrlParams& out_params) const
{
	out_params.mCtrlParamFile = GetCtrlParamFile();
	out_params.mChar = mChar;
	out_params.mGravity = GetGravity();
	out_params.mGround = mGround;

	out_params.mNetFiles = mCtrlParams.mNetFiles;
	out_params.mNetFiles[cTerrainRLCtrlFactory::eNetFileActor] = mCoachPoliNetFile;
	out_params.mNetFiles[cTerrainRLCtrlFactory::eNetFileCritic] = mCoachCriticNetFile;
	out_params.mNetFiles[cTerrainRLCtrlFactory::eNetFileActorModel] = mCoachPoliModelFile;
	out_params.mNetFiles[cTerrainRLCtrlFactory::eNetFileCriticModel] = mCoachCriticModelFile;
}

bool cScenarioExpMimic::BuildCoachController(const cTerrainRLCtrlFactory::tCtrlParams& params, std::shared_ptr<cTerrainRLCharController>& out_ctrl)
{
	std::shared_ptr<cCharController> ctrl;
	bool succ = cTerrainRLCtrlFactory::BuildController(params, ctrl);
	out_ctrl = std::static_pointer_cast<cTerrainRLCharController>(ctrl);
	return succ;
}

bool cScenarioExpMimic::EnableRandInitAction() const
{
	return false;
}

void cScenarioExpMimic::ApplyCoachBlend(double blend)
{
	double coach_weight = blend;
	double char_weight = 1 - blend;

	int char_ctrl_id = GetCharCtrlID();
	int coach_ctrl_id = GetCoachCtrlID();
	auto mix_ctrl = std::dynamic_pointer_cast<cMixController>(mChar->GetController());

	mix_ctrl->SetWeight(char_ctrl_id, char_weight);
	mix_ctrl->SetWeight(coach_ctrl_id, coach_weight);
}

void cScenarioExpMimic::UpdateCharacter(double time_step)
{
	UpdateKinChar(time_step);
	UpdateTrackController();
	cScenarioExp::UpdateCharacter(time_step);

#if defined(ENABLE_COACH_TAU_AVG)
	FetchCoachAction(time_step, mCoachActionSample);
#else
	FetchCoachAction(time_step, mCoachAction);
#endif
	
}

void cScenarioExpMimic::UpdateKinChar(double time_step)
{
	mKinChar->Update(time_step);
}

void cScenarioExpMimic::UpdateTrackController()
{
	int coach_ctrl_id = GetCoachCtrlID();
	std::shared_ptr<cMixController> mix_ctrl = std::static_pointer_cast<cMixController>(mChar->GetController());

	auto track_ctrl = std::dynamic_pointer_cast<cCtTrackController>(mix_ctrl->GetController(coach_ctrl_id));
	if (track_ctrl != nullptr)
	{
		Eigen::VectorXd pose;
		Eigen::VectorXd vel;

		double kin_time = mKinChar->GetTime();
		double ctrl_period = track_ctrl->GetUpdatePeriod();
		double next_time = kin_time;// +ctrl_period;
		mKinChar->CalcPose(next_time, pose);
		mKinChar->CalcVel(next_time, vel);
		track_ctrl->SetTargetPoseVel(pose, vel);
	}
}

void cScenarioExpMimic::FetchCoachAction(double time_step, Eigen::VectorXd& out_action)
{
	int coach_ctrl_id = GetCoachCtrlID();
	std::shared_ptr<cMixController> mix_ctrl = std::static_pointer_cast<cMixController>(mChar->GetController());
	auto coach_ctrl = mix_ctrl->GetController(coach_ctrl_id);

	switch (mMimicMode)
	{
	case eMimicModeTau:
		out_action = mix_ctrl->GetTau(coach_ctrl_id);
		break;
	case eMimicModeAction:
		coach_ctrl->RecordPoliAction(out_action);
		break;
	default:
		break;
	}
	
}

void cScenarioExpMimic::HandleNewActionUpdate()
{
	cScenarioExp::HandleNewActionUpdate();
	NewActionUpdateCoach();
}

void cScenarioExpMimic::NewActionUpdateCoach()
{
	mCoachActionCount = 0;

	if (IsCoachWarmup())
	{
		ApplyCoachBlend(1);
	}
	else if (mEnableCoachActiveProb)
	{
		double blend = (mRand.FlipCoin(mCoachActiveProb)) ? mCoachBlend : 0;
		ApplyCoachBlend(blend);
	}
}

void cScenarioExpMimic::PreSubstepUpdate(double time_step)
{
	cScenarioExp::PreSubstepUpdate(time_step);
#if defined(ENABLE_COACH_TAU_AVG)
	if (mCoachActionCount == 0)
	{
		mCoachAction = mCoachActionSample;
	}
	else
	{
		cMathUtil::AddAverage(mCoachAction, mCoachActionCount, mCoachActionSample, 1, mCoachAction);
	}

	++mCoachActionCount;
	RecordAction(mCurrTuple.mAction);
#endif
}

void cScenarioExpMimic::PostSubstepUpdate(double time_step)
{
	cScenarioExp::PostSubstepUpdate(time_step);
	UpdateCoachWarmupCounter(time_step);
}

int cScenarioExpMimic::GetNumWarmupCycles() const
{
	return gNumWarmupCycles;
}

void cScenarioExpMimic::IncCycleCount()
{
	if (!IsCoachWarmup())
	{
		cScenarioExp::IncCycleCount();
	}
}

std::shared_ptr<const cNNController> cScenarioExpMimic::GetNNController() const
{
	int char_ctrl_id = GetCharCtrlID();
	std::shared_ptr<cMixController> mix_ctrl = std::static_pointer_cast<cMixController>(mChar->GetController());
	std::shared_ptr<cNNController> ctrl = std::static_pointer_cast<cNNController>(mix_ctrl->GetController(char_ctrl_id));
	return ctrl;
}

void cScenarioExpMimic::RecordAction(Eigen::VectorXd& out_action) const
{
	int char_ctrl_id = GetCharCtrlID();
	std::shared_ptr<cMixController> mix_ctrl = std::static_pointer_cast<cMixController>(mChar->GetController());
	std::shared_ptr<cNNController> ctrl = std::static_pointer_cast<cNNController>(mix_ctrl->GetController(char_ctrl_id));

	int poli_action_size = ctrl->GetPoliActionSize();
	int start_idx = static_cast<int>(mCoachAction.size()) - poli_action_size;
	out_action = mCoachAction.segment(start_idx, poli_action_size);
}

double cScenarioExpMimic::CalcReward() const
{
	return 0;
}

bool cScenarioExpMimic::IsCoachWarmup() const
{
	return mCoachWarmupTimer < mCouchWarmupTimeMax;
}

void cScenarioExpMimic::ResetCoachWarmupCounter()
{
	mCouchWarmupTimeMax = mRand.RandDouble(0, mCoachWarmupMax);
	mCoachWarmupTimer = 0;
}

void cScenarioExpMimic::UpdateCoachWarmupCounter(double time_step)
{
	mCoachWarmupTimer += time_step;
	mCoachWarmupTimer = std::min(mCouchWarmupTimeMax, mCoachWarmupTimer);
}

double cScenarioExpMimic::GetEpisodeMaxTime() const
{
	return mEpisodeMaxTime + mCouchWarmupTimeMax;
}
