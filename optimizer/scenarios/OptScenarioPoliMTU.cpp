#include "OptScenarioPoliMTU.h"
#include "util/FileUtil.h"
#include "learning/ACTrainer.h"

cOptScenarioPoliMTU::cOptScenarioPoliMTU(cOptimizer& optimizer) 
	: mOptimizer(optimizer)
{
	mTimeStep = 1 / 30.0;
	
	mPolicyOutputFile = "";
	mMTUOutputFile = "";
	mIntPolicyOutputFile = "";
	mIntMTUOutputFile = "";

	mNumTrainThreads = 1;
	mNumOptThreads = 1;

	mMaxIters = 8;
	mPoolSize = 1;
	mTimeStep = cOptScenario::gTimeStep;
	mIter = 0;

	ResetParams();
}

cOptScenarioPoliMTU::~cOptScenarioPoliMTU()
{
}

void cOptScenarioPoliMTU::Init()
{
	cScenario::Init();

	ResetParams();
	BuildTrainScene(mTrainScene);
	BuildOptScene(mOptScene);
	SetupScenes();
}

void cOptScenarioPoliMTU::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenario::ParseArgs(parser);
	mArgParser = parser;

	parser->ParseInt("poli_mtu_max_iters", mMaxIters);

	parser->ParseString("output_policy_path", mPolicyOutputFile);
	parser->ParseString("output_mtu_path", mMTUOutputFile);
	parser->ParseString("int_output_policy_path", mIntPolicyOutputFile);
	parser->ParseString("int_output_mtu_path", mIntMTUOutputFile);

	parser->ParseInt("num_train_threads", mNumTrainThreads);
	parser->ParseInt("num_opt_threads", mNumOptThreads);
}

void cOptScenarioPoliMTU::Reset()
{
	cScenario::Reset();
	ResetParams();

	mTrainScene->Reset();
	mOptScene->Reset();
}

void cOptScenarioPoliMTU::Clear()
{
	cScenario::Clear();

	ResetParams();
	mTrainScene->Clear();
	mOptScene->Clear();
	mTrainScene.reset();
	mOptScene.reset();
}

void cOptScenarioPoliMTU::Run()
{
	for (mIter = 0; mIter < mMaxIters; ++mIter)
	{
		printf("Poli MTU Iter: %i\n", mIter + 1);
		TrainPolicy();
		OptmizeMTU();
	}
}

void cOptScenarioPoliMTU::SetTimeStep(double time_step)
{
	mTimeStep = time_step;
}

std::string cOptScenarioPoliMTU::GetName() const
{
	return "Opt Policy MTU";
}

void cOptScenarioPoliMTU::BuildTrainScene(std::shared_ptr<cScenarioImitate>& out_scene) const
{
	cScenarioImitate* scene = new cScenarioImitate();
	scene->SetTimeStep(mTimeStep);
	scene->SetExpPoolSize(mNumTrainThreads);
	out_scene = std::shared_ptr<cScenarioImitate>(scene);
}

void cOptScenarioPoliMTU::BuildOptScene(std::shared_ptr<cOptScenarioMTU>& out_scene) const
{
	cOptScenarioMTU* scene = new cOptScenarioMTU(mOptimizer);
	scene->SetPoolSize(mNumOptThreads);
	out_scene = std::shared_ptr<cOptScenarioMTU>(scene);
}

void cOptScenarioPoliMTU::SetupScenes()
{
	mTrainScene->ParseArgs(mArgParser);
	mOptScene->ParseArgs(mArgParser);

	mTrainScene->SetOutputFile(mPolicyOutputFile);
	mOptScene->SetOutputPath(mMTUOutputFile);
}

void cOptScenarioPoliMTU::ResetParams()
{
	mIter = 0;
}

bool cOptScenarioPoliMTU::HasInitPolicy() const
{
	const std::vector<std::string>& net_files = mTrainScene->GetNetFiles();
	return net_files[cTerrainRLCtrlFactory::eNetFileActorModel] != "";
}

void cOptScenarioPoliMTU::TrainPolicy()
{
	bool has_init_policy = HasInitPolicy();
	if (!has_init_policy)
	{
		cNeuralNetTrainer::tParams& trainer_params = mTrainScene->GetTrainerParams();
		trainer_params.mInitInputOffsetScale = true;
	}

	mTrainScene->Init();

	if (mIter == 0)
	{
		OutputIntPoliModel(0, mIntPolicyOutputFile);
	}
	else if (mIter > 0)
	{
		bool succ = mTrainScene->LoadControlParams(mMTUOutputFile);
		assert(succ); // load mtu params
	}

	mTrainScene->Run();

	bool update_poli_models = (mIter == 0);
	if (update_poli_models)
	{
		const std::string& model_file = mTrainScene->GetOutputFile();
		std::string critic_file = cACTrainer::GetCriticFilename(model_file);

		cNeuralNetTrainer::tParams& trainer_params = mTrainScene->GetTrainerParams();
		trainer_params.mInitInputOffsetScale = false;

		std::vector<std::string>& net_files = mTrainScene->GetNetFiles();
		net_files[cTerrainRLCtrlFactory::eNetFileActorModel] = model_file;
		net_files[cTerrainRLCtrlFactory::eNetFileCriticModel] = critic_file;

		mOptScene->SetModelFile(model_file);
		mOptScene->SetCriticModelFile(critic_file);
	}

	OutputIntPoliModel(mIter + 1, mIntPolicyOutputFile);
	mTrainScene->Clear();
}

void cOptScenarioPoliMTU::OptmizeMTU()
{
	mOptScene->Init();

	if (mIter == 0)
	{
		OutputIntMTU(0, mIntMTUOutputFile);
	}
	else if (mIter > 0)
	{
		bool succ = mOptScene->LoadControlParams(mMTUOutputFile);
		assert(succ);
	}

	mOptScene->Run();
	OutputIntMTU(mIter + 1, mIntMTUOutputFile);

	mOptScene->Clear();
}

void cOptScenarioPoliMTU::OutputIntPoliModel(int iter, const std::string& output_path) const
{
	std::string ext = cFileUtil::GetExtension(output_path);
	std::string no_ext = cFileUtil::RemoveExtension(output_path);
	char suffix[64];
	sprintf(suffix, "%04d", iter);
	std::string int_out_path = no_ext + "_" + suffix + "." + ext;

	OutputPoliModel(int_out_path);
}

void cOptScenarioPoliMTU::OutputIntMTU(int iter, const std::string& output_path) const
{
	std::string ext = cFileUtil::GetExtension(output_path);
	std::string no_ext = cFileUtil::RemoveExtension(output_path);
	char suffix[64];
	sprintf(suffix, "%04d", iter);
	std::string int_out_path = no_ext + "_" + suffix + "." + ext;

	OutputMTU(int_out_path);
}

void cOptScenarioPoliMTU::OutputPoliModel(const std::string& output_path) const
{
	mTrainScene->OutputModel(output_path);
}

void cOptScenarioPoliMTU::OutputMTU(const std::string& output_path) const
{
	mOptScene->OutputSolution(output_path);
}