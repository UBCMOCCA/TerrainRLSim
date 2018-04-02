/*
 * CaclaFDTrainer.cpp
 *
 *  Created on: Nov 18, 2016
 *      Author: Glen
 */

#include "CaclaFDTrainer.h"
#include "util/FileUtil.h"
#include "ACDLearner.h"

cCaclaFDTrainer::cCaclaFDTrainer() : cCaclaTrainer() {
	// TODO Auto-generated constructor stub

}

cCaclaFDTrainer::~cCaclaFDTrainer() {
	// TODO Auto-generated destructor stub
}

void cCaclaFDTrainer::Init(const tParams& params)
{
	cCaclaTrainer::Init(params);
	InitForwardDynamicsProblem(mForwardDynamicsProb);
}

void cCaclaFDTrainer::RequestLearner(std::shared_ptr<cNeuralNetLearner>& out_learner)
{
	out_learner = std::shared_ptr<cACDLearner>(new cACDLearner(shared_from_this()));
}

void cCaclaFDTrainer::InitInputOffsetScaleTypes()
{
	cACTrainer::InitInputOffsetScaleTypes();
	InitForwardDynamicsInputOffsetScaleTypes();
}

int cCaclaFDTrainer::GetForwardDynamicsOutputSize() const
{
	const auto& curr_net = GetForwardDynamics();
	return curr_net->GetOutputSize();
}

int cCaclaFDTrainer::GetForwardDynamicsInputSize() const
{
	const auto& curr_net = GetForwardDynamics();
	return curr_net->GetInputSize();
}

void cCaclaFDTrainer::InitForwardDynamicsInputOffsetScaleTypes()
{
	int input_size = GetForwardDynamicsInputSize();
	mForwardDynamicsInputOffsetScaleTypes.resize(input_size);
	for (int i = 0; i < input_size; ++i)
	{
		mForwardDynamicsInputOffsetScaleTypes[i] = cNeuralNet::eOffsetScaleTypeNone;
	}
}

void cCaclaFDTrainer::SetInputOffsetScaleType(const std::vector<cNeuralNet::eOffsetScaleType>& scale_types)
{
	cACTrainer::SetInputOffsetScaleType(scale_types);
	// 
	SetForwardDynamicsInputOffsetScaleType(scale_types);
}

void cCaclaFDTrainer::SetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	cACTrainer::SetInputOffsetScale(offset, scale);
	const auto& actor = GetActor();
	Eigen::VectorXd scale_ = actor->GetOutputScale();
	Eigen::VectorXd offset_ = actor->GetOutputOffset();
	Eigen::VectorXd _scale = Eigen::VectorXd::Ones(scale.rows()+ scale_.rows());
	Eigen::VectorXd _offset = Eigen::VectorXd::Ones(offset.rows() + offset_.rows());
	_scale.segment(0, scale.rows()) = scale;
	_scale.segment(scale.rows(), scale_.rows()) = scale_;
	_offset.segment(0, offset.rows()) = offset;
	_offset.segment(offset.rows(), offset_.rows()) = offset_;
	SetForwardDynamicsInputOffsetScale(_offset, _scale);
}

void cCaclaFDTrainer::SetOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	cACTrainer::SetOutputOffsetScale(offset, scale);
	const auto& actor = GetActor();
	Eigen::VectorXd scale_ = actor->GetInputScale();
	Eigen::VectorXd offset_ = actor->GetInputOffset();
	SetForwardDynamicsOutputOffsetScale(offset_, scale_);
}

void cCaclaFDTrainer::SetForwardDynamicsInputOffsetScaleType(const std::vector<cNeuralNet::eOffsetScaleType>& scale_types)
{
	assert(scale_types.size() == GetForwardDynamicsInputSize());
	mForwardDynamicsInputOffsetScaleTypes = scale_types;
}

void cCaclaFDTrainer::SetForwardDynamicsInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	const auto& fd = GetForwardDynamics();
	fd->SetInputOffsetScale(offset, scale);
}

void cCaclaFDTrainer::SetForwardDynamicsOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	const auto& fd = GetForwardDynamics();
	fd->SetOutputOffsetScale(offset, scale);
}

void cCaclaFDTrainer::UpdateOffsetScale()
{
	cACTrainer::UpdateOffsetScale();
	UpdateForwardDynamicsOffsetScale();
}

void cCaclaFDTrainer::UpdateForwardDynamicsOffsetScale()
{
	// just use the actor for this data computation
	Eigen::VectorXd offset = GetActorInputOffset();
	Eigen::VectorXd scale = GetActorInputScale();
	mDataRecordActorX.CalcOffsetScale(mActorInputOffsetScaleTypes, mParams.mInputScaleMax, offset, scale);
	
	// Keep old action input scales
	const auto& fd = GetForwardDynamics();
	Eigen::VectorXd scale_ = fd->GetInputScale();
	Eigen::VectorXd offset_ = fd->GetInputOffset();
	const auto& actor = GetActor();
	Eigen::VectorXd scale_o = actor->GetOutputScale();
	Eigen::VectorXd offset_o = actor->GetOutputOffset();
	scale_.segment(0, scale.rows()) = scale;
	scale_.segment(scale.rows(), scale_o.rows()) = scale_o;
	offset_.segment(0, offset.rows()) = offset;
	offset_.segment(offset.rows(), offset_o.rows()) = offset_o;
	SetForwardDynamicsInputOffsetScale(offset_, scale_);
	SetForwardDynamicsOutputOffsetScale(offset, scale);

	if (EnableAsyncMode())
	{
		// UpdateParamServerForwardDyanmicsInputOffsetScale(offset, scale);
	}
}

void cCaclaFDTrainer::InitForwardDynamicsProblem(cNeuralNet::tProblem& out_prob) const
{
	const auto& curr_net = GetForwardDynamics();
	const int x_size = curr_net->GetInputSize();
	const int y_size = curr_net->GetOutputSize();
	int num_data = GetBatchSize();

	out_prob.mX.resize(num_data, x_size);
	out_prob.mY.resize(num_data, y_size);
	out_prob.mPassesPerStep = 1;
}

void cCaclaFDTrainer::OutputModel(const std::string& filename) const
{
	cCaclaTrainer::OutputModel(filename);

	std::string fd_filename = GetForwardDynamicsFilename(filename);
	OutputForwardDynamics(fd_filename);
}

const std::string& cCaclaFDTrainer::GetForwardDynamicsNetFile() const
{
	return mParams.mForwardDynamicsNetFile;
}

const std::string& cCaclaFDTrainer::GetForwardDynamicsSolverFile() const
{
	return mParams.mForwardDynamicsSolverFile;
}

const std::unique_ptr<cNeuralNet>& cCaclaFDTrainer::GetForwardDynamics() const
{
	return mForwardDynamicsNet;
}

std::string cCaclaFDTrainer::GetForwardDynamicsFilename(const std::string& actor_filename) const
{
	std::string file_no_ext = cFileUtil::RemoveExtension(actor_filename);
	std::string ext = cFileUtil::GetExtension(actor_filename);
	std::string critic_file = file_no_ext + "_forward_dynamics." + ext;
	return critic_file;
}

void cCaclaFDTrainer::OutputIntermediateModel(const std::string& filename) const
{
	cCaclaTrainer::OutputIntermediateModel(filename);

	std::string fd_filename = GetForwardDynamicsFilename(filename);
	OutputForwardDynamics(fd_filename);
}

void cCaclaFDTrainer::OutputForwardDynamics(const std::string& filename) const
{
	const auto& actor = GetForwardDynamics();
	actor->OutputModel(filename);
	printf("ForwardDynamics model saved to %s\n", filename.c_str());
}

void cCaclaFDTrainer::BuildNets()
{
	int pool_size = GetPoolSize();
	BuildNetPool(GetCriticNetFile(), GetCriticSolverFile(), pool_size);
	BuildActor(GetActorNetFile(), GetActorSolverFile());
	BuildForwardDynamics(GetForwardDynamicsNetFile(), GetForwardDynamicsSolverFile());
	LoadModels();
}

void cCaclaFDTrainer::BuildForwardDynamics(const std::string& net_file, const std::string& solver_file)
{
	mForwardDynamicsNet = std::unique_ptr<cNeuralNet>(new cNeuralNet());
	mForwardDynamicsNet->LoadNet(net_file);
	mForwardDynamicsNet->LoadSolver(solver_file);
}

void cCaclaFDTrainer::LoadModels()
{
	if (mParams.mModelFile != "")
	{
		LoadActorModel(mParams.mModelFile);
	}

	if (mParams.mCriticModelFile != "")
	{
		LoadCriticModel(mParams.mCriticModelFile);
	}

	if (mParams.mForwardDynamicsModelFile != "")
	{
		LoadForwardDynamicsModel(mParams.mForwardDynamicsModelFile);
	}
}

void cCaclaFDTrainer::LoadForwardDynamicsModel(const std::string& model_file)
{
	const auto& actor = GetForwardDynamics();
	actor->LoadModel(model_file);
}

bool cCaclaFDTrainer::Step()
{
	cCaclaTrainer::Step();
	UpdateForwardDynamics();
	return true;
}

bool cCaclaFDTrainer::BuildForwardDynamicsProblem(cNeuralNet::tProblem& out_prob)
{
	bool succ = true;
	int num_data = GetNumTuplesPerBatch();
	FetchMinibatch(num_data, mBatchBuffer);
	cNeuralNet::tProblem out_prob_action;

	if (mBatchBuffer.size() >= num_data)
	{
		{
#if defined(OUTPUT_TRAINER_LOG)
			TIMER_RECORD_BEG(BUILD_TUPLE_X)
#endif
			BuildProblemStateAndAction(0, mBatchBuffer, out_prob);
#if defined(OUTPUT_TRAINER_LOG)
			{
				std::lock_guard<std::mutex> lock(mLogLock);
				TIMER_RECORD_END(BUILD_TUPLE_X, mLog.mBuildTupleXTime, mLog.mBuildTupleXSamples)
			}
#endif
		}

		{
#if defined(OUTPUT_TRAINER_LOG)
			TIMER_RECORD_BEG(BUILD_TUPLE_Y)
#endif
				BuildProblemNextState( 0, mBatchBuffer, out_prob.mX, out_prob);
#if defined(OUTPUT_TRAINER_LOG)
			{
				std::lock_guard<std::mutex> lock(mLogLock);
				TIMER_RECORD_END(BUILD_TUPLE_Y, mLog.mBuildTupleYTime, mLog.mBuildTupleYSamples)
			}
#endif
		}

		UpdateMisc(mBatchBuffer);
}
	else
	{
		succ = false;
	}

	return succ;
}

void cCaclaFDTrainer::StepForwardDynamics()
{
#if defined(OUTPUT_TRAINER_LOG)
	TIMER_RECORD_BEG(TRAIN_STEP_ACTOR)
#endif

	BuildForwardDynamicsProblem(mForwardDynamicsProb);
	printf("ForwardDynamics Iter %i\n", mForwardDynamicsIter);
	// std::cout << "X-length : " << mForwardDynamicsProb.mX.row(0).cols() << " X: " <<
	//	mForwardDynamicsProb.mX.row(0) << std::endl;
	// std::cout << "****Y-length: " << mForwardDynamicsProb.mY.row(0).cols() << " Y: " << 
	//	mForwardDynamicsProb.mY.row(0) << std::endl;
	UpdateForwardDynamicsNet(mForwardDynamicsProb);
	IncForwardDynamicsIter();

#if defined(OUTPUT_TRAINER_LOG)
	{
		std::lock_guard<std::mutex> lock(mLogLock);
		TIMER_RECORD_END(TRAIN_STEP_ACTOR, mLog.mStepActorTime, mLog.mStepActorSamples)
	}
#endif
}

void cCaclaFDTrainer::UpdateForwardDynamics()
{ // This should use a batch like a critic update but perform a actor like net update

	StepForwardDynamics();
}

int cCaclaFDTrainer::GetServerForwardDynamicsID() const
{
	return mParams.mPoolSize;
}

void cCaclaFDTrainer::UpdateForwardDynamicsNet(const cNeuralNet::tProblem& prob)
{ // Should perform a actor like update, not using the actor buffer
#if defined(OUTPUT_TRAINER_LOG)
	TIMER_RECORD_BEG(UPDATE_ACTOR_NET)
#endif

	auto& curr_net = mForwardDynamicsNet;
	if (EnableAsyncMode())
	{
		int net_id = GetServerForwardDynamicsID();

		double loss = curr_net->ForwardBackward(prob);
		printf("ForwardDynamics Net Loss: %.8f\n", loss);

		cParamServer::tInputInfo server_input;
		server_input.mID = net_id;
		server_input.mGradNet = curr_net.get();

		cParamServer::tOutputInfo server_output;
		server_output.mSyncNet = curr_net.get();

		mParamServer->UpdateNet(server_input, server_output);
		mActorIter = server_output.mIter;
	}
	else
	{
		curr_net->Train(prob);
	}

#if defined(OUTPUT_TRAINER_LOG)
	{
		std::lock_guard<std::mutex> lock(mLogLock);
		TIMER_RECORD_END(UPDATE_ACTOR_NET, mLog.mUpdateActorNetTime, mLog.mUpdateActorNetSamples)
	}
#endif
}

bool cCaclaFDTrainer::HasForwardDynamicsInitModel() const
{
	bool has_init_model = false;
	const auto& actor = GetForwardDynamics();
	if (actor != nullptr)
	{
		has_init_model = actor->HasValidModel();
	}
	return has_init_model;
}

void cCaclaFDTrainer::EvalForwardDynamics(const tExpTuple& tuple, Eigen::VectorXd& out_y)
{
	Eigen::VectorXd xState;
	Eigen::VectorXd xAction;
	BuildTupleX(tuple, xState);
	BuildTupleY(0, tuple, xAction);
	Eigen::VectorXd xOutState = Eigen::VectorXd::Ones(xState.rows() + xAction.rows());
	xOutState.segment(0, xState.rows()) = xState;
	xOutState.segment(xState.rows(), xAction.rows()) = xAction;
	const auto& net = GetForwardDynamics();
	net->Eval(xOutState, out_y);
}

void cCaclaFDTrainer::IncForwardDynamicsIter()
{
	if (!EnableAsyncMode())
	{
		++mForwardDynamicsIter;
	}
}

/*
void cCaclaFDTrainer::UpdateCritic()
{
	for (int i = 0; i < GetNetPoolSize(); ++i)
	{
		printf("Update Net %i:\n", i);
		bool succ = BuildProblem(i, mProb);
		if (succ)
		{
			UpdateNet(i, mProb);
		}
	}
}
*/
/*
void cCaclaFDTrainer::UpdateParamServerForwardDynamicsInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	assert(EnableAsyncMode());
	mParamServer->UpdateInputOffsetScale(GetServerForwardDynamicsID(), offset, scale);
}
*/

void cCaclaFDTrainer::SyncNets()
{
	cCaclaTrainer::SyncNets();
	SyncForwardDynamicsNet();
}

void cCaclaFDTrainer::SyncForwardDynamicsNet()
{
	int net_id = GetServerForwardDynamicsID();
	auto& curr_net = mForwardDynamicsNet;
	mParamServer->SyncNet(net_id, *curr_net);
}

void cCaclaFDTrainer::ResetSolvers()
{
	cCaclaTrainer::ResetSolvers();
	ResetForwardDynamicsSolver();
}

void cCaclaFDTrainer::ResetForwardDynamicsSolver()
{
	if (EnableAsyncMode())
	{
		mParamServer->ResetSolver(GetServerForwardDynamicsID());
	}
	else
	{
		mForwardDynamicsNet->ResetSolver();
	}
}

void cCaclaFDTrainer::UpdateParamServerForwardDynamicsInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	assert(EnableAsyncMode());
	mParamServer->UpdateInputOffsetScale(GetServerForwardDynamicsID(), offset, scale);
}

void cCaclaFDTrainer::BuildProblemStateAndAction(int net_id, const std::vector<int>& tuple_ids, cNeuralNet::tProblem& out_prob)
{
	int num_data = static_cast<int>(tuple_ids.size());
	assert(num_data == GetBatchSize());
	assert(out_prob.mX.rows() == num_data);

	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);

		Eigen::VectorXd xState;
		Eigen::VectorXd xAction;
		BuildTupleX(tuple, xState);
		BuildTupleActorY(tuple, xAction);
		Eigen::VectorXd xOutState = Eigen::VectorXd::Ones(xState.rows() + xAction.rows());
		xOutState.segment(0, xState.rows()) = xState;
		xOutState.segment(xState.rows(), xAction.rows()) = xAction;

		// std::cout << "xState: " << xState.rows() << " data: " << xState << std::endl;
		// std::cout << "xAction: " << xAction.rows() << " data: " << xAction << std::endl;
		// std::cout << "xState+xAction: " << xOutState.rows() << " data: " << xOutState.transpose() << std::endl;
		out_prob.mX.row(i) = xOutState;
	}
}

void cCaclaFDTrainer::BuildProblemNextState(int net_id, const std::vector<int>& tuple_ids,
	const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob)
{
	int num_data = static_cast<int>(tuple_ids.size());
	assert(num_data == GetNumTuplesPerBatch());
	assert(out_prob.mY.rows() == GetBatchSize());

	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);

		Eigen::VectorXd y;
		BuildTupleNextState(net_id, tuple, y);
		out_prob.mY.row(i) = y;
	}
}
