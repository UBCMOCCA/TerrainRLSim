#include "OptScenarioMTU.h"
#include "scenarios/ScenarioImitateEval.h"
#include "sim/CtMTUController.h"

cOptScenarioMTU::cOptScenarioMTU(cOptimizer& optimizer)
	: cOptScenarioSimChar(optimizer)
{
	mMaxEpisodes = 1;
	mModelFile = "";
	mCriticModelFile = "";
}

cOptScenarioMTU::~cOptScenarioMTU()
{
}

void cOptScenarioMTU::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cOptScenarioSimChar::ParseArgs(parser);
	
	parser->ParseInt("poli_eval_max_episodes", mMaxEpisodes);

	mTargetMTUIDs.clear();
	mMirrorParams.clear();
	parser->ParseIntArray("opt_mtu_target_ids", mTargetMTUIDs);

	ParseMirrorParams(parser, mMirrorParams);
}

void cOptScenarioMTU::Init()
{
	cOptScenarioSimChar::Init();
	SetupTargetMTUs();
	InitPt(mSoln);
}


int cOptScenarioMTU::GetDimensions() const
{
	int num_params = mNumOptParams;
	return num_params;
}

void cOptScenarioMTU::InitPt(tPoint& pt) const
{
	const auto& ctrl = GetDefaultController();

	Eigen::VectorXd full_params;
	ctrl->BuildOptParams(full_params);
	FetchTargetOptParams(full_params, pt);
}

void cOptScenarioMTU::InitScale(tPoint& scale) const
{
	const auto& controller = GetDefaultController();

	Eigen::VectorXd full_scale;
	controller->FetchOptParamScale(full_scale);
	FetchTargetOptParams(full_scale, scale);
}

void cOptScenarioMTU::SetModelFile(const std::string& model_file)
{
	mModelFile = model_file;

	if (mModelFile != "")
	{
		for (size_t i = 0; i < mScenePool.size(); ++i)
		{
			const auto& scene = std::dynamic_pointer_cast<cScenarioPoliEval>(mScenePool[i]);
			assert(scene != nullptr);
			scene->SetPoliModelFile(mModelFile);
		}
	}
}

void cOptScenarioMTU::SetCriticModelFile(const std::string& model_file)
{
	mCriticModelFile = model_file;

	if (mCriticModelFile != "")
	{
		for (size_t i = 0; i < mScenePool.size(); ++i)
		{
			const auto& scene = std::dynamic_pointer_cast<cScenarioPoliEval>(mScenePool[i]);
			assert(scene != nullptr);
			scene->SetCriticModelFile(model_file);
		}
	}
}

bool cOptScenarioMTU::IsThreadSafe() const
{
	return true;
}

void cOptScenarioMTU::OutputPt(FILE* f, const tPoint& pt) const
{
	const auto& controller = GetDefaultController();
	Eigen::VectorXd full_params;
	BuildFullParams(pt, full_params);
	controller->OutputOptParams(f, full_params);
}

std::string cOptScenarioMTU::GetName() const
{
	return "Optimize MTU";
}

void cOptScenarioMTU::BuildScene(int id, std::shared_ptr<cScenarioSimChar>& out_scene)
{
	out_scene = std::shared_ptr<cScenarioImitateEval>(new cScenarioImitateEval());
}

void cOptScenarioMTU::InitScenePool()
{
	SetModelFile(mModelFile);
	SetCriticModelFile(mCriticModelFile);

	cOptScenarioSimChar::InitScenePool();
}

void cOptScenarioMTU::SetupTargetMTUs()
{
	auto ctrl = std::dynamic_pointer_cast<cCtMTUController>(GetDefaultController());
	assert(ctrl != nullptr);
	if (mTargetMTUIDs.size() == 0)
	{
		int num_mtus = ctrl->GetNumMTUs();
		mTargetMTUIDs.resize(num_mtus);

		for (int i = 0; i < num_mtus; ++i)
		{
			mTargetMTUIDs[i] = i;
		}
	}

	int num_targets = static_cast<int>(mTargetMTUIDs.size());
	mMTUParamOffsets.resize(num_targets);
	mNumOptParams = 0;

	for (int i = 0; i < num_targets; ++i)
	{
		int id = mTargetMTUIDs[i];
		int offset = ctrl->CalcOptParamOffset(id);
		int size = ctrl->GetNumOptParams(id);
		mMTUParamOffsets[i] = offset;
		mNumOptParams += size;
	}

	int num_mirror = static_cast<int>(mMirrorParams.size());
	for (int i = 0; i < num_mirror; ++i)
	{
		tMirrorParam& curr_params = mMirrorParams[i];
		curr_params.mSrcOffset = ctrl->CalcOptParamOffset(curr_params.mSrcID);
		curr_params.mDstOffset = ctrl->CalcOptParamOffset(curr_params.mDstID);
	}

	ctrl->BuildOptParams(mDefaultFullParams);
}

int cOptScenarioMTU::GetNumTargetMTUs() const
{
	return static_cast<int>(mTargetMTUIDs.size());
}

void cOptScenarioMTU::ParseMirrorParams(const std::shared_ptr<cArgParser>& parser, std::vector<tMirrorParam>& out_params) const
{
	std::vector<int> src_ids;
	std::vector<int> dst_ids;

	parser->ParseIntArray("opt_mtu_mirror_srcs", src_ids);
	parser->ParseIntArray("opt_mtu_mirror_dsts", dst_ids);
	assert(src_ids.size() == dst_ids.size());

	int num_ids = static_cast<int>(src_ids.size());
	out_params.resize(num_ids);
	for (int i = 0; i < num_ids; ++i)
	{
		tMirrorParam& params = out_params[i];
		params.mSrcID = src_ids[i];
		params.mDstID = dst_ids[i];
		params.mSrcOffset = gInvalidIdx;
		params.mDstOffset = gInvalidIdx;
	}
}

double cOptScenarioMTU::CalcObjVal(const std::shared_ptr<cScenarioSimChar>& scene)
{
	const double obj_scale = 0.01;

	auto eval_scene = std::dynamic_pointer_cast<cScenarioImitateEval>(scene);
	double max_time = eval_scene->GetEpisodeMaxTime();
	double avg_reward = eval_scene->CalcAvgCumulativeReward();

	double obj_val = obj_scale * avg_reward / max_time;
	obj_val = 1 - obj_val;

	return obj_val;
}

const std::shared_ptr<cCharController>& cOptScenarioMTU::GetDefaultController() const
{
	const std::shared_ptr<cScenarioSimChar>& scene = GetDefaultScene();
	const auto& character = scene->GetCharacter();
	const std::shared_ptr<cCharController>& controller = character->GetController();
	return controller;
}

double cOptScenarioMTU::EvalPt(const tPoint& pt, const std::shared_ptr<cScenarioSimChar>& scene)
{
	auto eval_scene = std::dynamic_pointer_cast<cScenarioPoliEval>(scene);
	eval_scene->ResetRecord();
	eval_scene->Reset();
	
	SetOptParams(pt, scene);

	double curr_obj_val = 0;
	while (!CheckTermination(scene))
	{
		UpdateScene(gTimeStep, scene);
	}

	double obj_val = CalcObjVal(scene);
	return obj_val;
}

void cOptScenarioMTU::SetOptParams(const tPoint& pt, const std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	const auto& sim_char = out_scene->GetCharacter();
	auto& controller = sim_char->GetController();

	Eigen::VectorXd full_params;
	BuildFullParams(pt, full_params);
	controller->SetOptParams(full_params);
}

bool cOptScenarioMTU::CheckTermination(const std::shared_ptr<cScenarioSimChar>& scene) const
{
	auto eval_scene = std::dynamic_pointer_cast<cScenarioPoliEval>(scene);
	int num_episodes = eval_scene->GetNumEpisodes();
	bool is_end = num_episodes >= mMaxEpisodes;
	return is_end;
}

void cOptScenarioMTU::FetchTargetOptParams(const Eigen::VectorXd& full_params, Eigen::VectorXd& out_params) const
{
	auto ctrl = std::dynamic_pointer_cast<cCtMTUController>(GetDefaultController());
	assert(ctrl != nullptr);

	int dim = GetDimensions();
	int full_param_size = GetFullParamsSize();
	assert(static_cast<int>(full_params.size()) == full_param_size);

	out_params.resize(dim);

	int offset = 0;
	for (int i = 0; i < GetNumTargetMTUs(); ++i)
	{
		int id = mTargetMTUIDs[i];
		int full_offset = mMTUParamOffsets[i];
		int full_size = ctrl->GetNumOptParams(id);
		out_params.segment(offset, full_size) = full_params.segment(full_offset, full_size);
		offset += full_size;
	}
	assert(offset == dim);
}

void cOptScenarioMTU::ApplyTargetOptParams(const Eigen::VectorXd& opt_params, Eigen::VectorXd& out_full_params) const
{
	auto ctrl = std::dynamic_pointer_cast<cCtMTUController>(GetDefaultController());
	assert(ctrl != nullptr);
	
	int dim = GetDimensions();
	int full_param_size = GetFullParamsSize();
	assert(static_cast<int>(opt_params.size()) == dim);
	assert(static_cast<int>(out_full_params.size()) == full_param_size);

	int offset = 0;
	for (int i = 0; i < GetNumTargetMTUs(); ++i)
	{
		int id = mTargetMTUIDs[i];
		int full_offset = mMTUParamOffsets[i];
		int full_size = ctrl->GetNumOptParams(id);
		out_full_params.segment(full_offset, full_size) = opt_params.segment(offset, full_size);
		offset += full_size;
	}

	for (size_t i = 0; i < mMirrorParams.size(); ++i)
	{
		const tMirrorParam& mirror_param = mMirrorParams[i];

		int src_offset = mirror_param.mSrcOffset;
		int src_size = ctrl->GetNumOptParams(mirror_param.mSrcID);
		int dst_offset = mirror_param.mDstOffset;
		int dst_size = ctrl->GetNumOptParams(mirror_param.mDstID);
		assert(src_size == dst_size);

		out_full_params.segment(dst_offset, dst_size) = out_full_params.segment(src_offset, src_size);
	}
}

void cOptScenarioMTU::BuildFullParams(const Eigen::VectorXd& opt_params, Eigen::VectorXd& out_full_params) const
{
	out_full_params = mDefaultFullParams;
	ApplyTargetOptParams(opt_params, out_full_params);
}

int cOptScenarioMTU::GetFullParamsSize() const
{
	const auto& ctrl = GetDefaultController();
	return ctrl->GetNumOptParams();
}