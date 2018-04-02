#include "NeuralNetTrainer.h"
#include "util/FileUtil.h"
#include "util/Util.h"

const std::string gLogFile = "output/logs/trainer_log.txt";

void cNeuralNetTrainer::tDataRecord::Init(int size)
{
	int mCount = 0;
	mMin = std::numeric_limits<double>::infinity() * Eigen::VectorXd::Ones(size);
	mMax = -std::numeric_limits<double>::infinity() * Eigen::VectorXd::Ones(size);
	mMean = Eigen::VectorXd::Zero(size);
	mMeanSquares = Eigen::VectorXd::Zero(size);
}

void cNeuralNetTrainer::tDataRecord::Update(const Eigen::VectorXd data)
{
	mMin = mMin.cwiseMin(data);
	mMax = mMax.cwiseMax(data);
	mMean = (mCount / (mCount + 1.0)) * mMean + (1 / (mCount + 1.0)) * data;
	mMeanSquares = (mCount / (mCount + 1.0)) * mMeanSquares + (1 / (mCount + 1.0)) * data.cwiseProduct(data);
	++mCount;
}

void cNeuralNetTrainer::tDataRecord::CalcVar(Eigen::VectorXd& out_var) const
{
	out_var = mMeanSquares - mMean.cwiseProduct(mMean);
}

void cNeuralNetTrainer::tDataRecord::CalcOffsetScale(const std::vector<cNeuralNet::eOffsetScaleType>& scale_types, double max_scale,
									Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int size = GetSize();
	assert(scale_types.size() == size);
	assert(out_offset.size() == size);
	assert(out_scale.size() == size);

	Eigen::VectorXd var;
	CalcVar(var);

	for (int i = 0; i < size; ++i)
	{
		double curr_min = mMin[i];
		double curr_max = mMax[i];
		double curr_mean = mMean[i];
		double curr_var = var[i];

		double curr_offset = out_offset[i];
		double curr_scale = out_scale[i];
		cNeuralNet::eOffsetScaleType curr_type = scale_types[i];

		if (curr_type != cNeuralNet::eOffsetScaleTypeFixed)
		{
			if (curr_max == curr_min)
			{
				curr_offset = 0;
				curr_scale = 0;
			}
			else
			{
				curr_offset = -curr_mean;
				curr_scale = 1 / std::sqrt(curr_var);
				curr_scale = std::min(max_scale, curr_scale);
			}
		}

		out_offset[i] = curr_offset;
		out_scale[i] = curr_scale;
	}
}

int cNeuralNetTrainer::tDataRecord::GetSize() const
{
	return static_cast<int>(mMin.size());
}

double cNeuralNetTrainer::CalcDiscountNorm(double discount)
{
	double norm = (1 - discount);
	return norm;
}

cNeuralNetTrainer::cNeuralNetTrainer()
{
	ResetParams();
	mDone = false;
	mParamServer = nullptr;
	mAvgReward = 0.5;
}

cNeuralNetTrainer::~cNeuralNetTrainer()
{
}

void cNeuralNetTrainer::Init(const tParams& params)
{
	mParams = params;
	BuildNets();
	BuildExpBuffer(mExpBuffer);
	InitExpBuffer(params.mPlaybackMemSize);
	InitInputOffsetScaleTypes();

	ResetParams();
	InitBatchBuffer();
	InitProblem(mProb);
	InitDataRecord();

	if (EnableAsyncMode())
	{
		SyncNets();
	}
}

void cNeuralNetTrainer::Clear()
{
	ResetParams();

	mNetPool.clear();
	mExpBuffer->Clear();
	mBatchBuffer.clear();
	mProb.Clear();
	mLearners.clear();
}

void cNeuralNetTrainer::LoadModel(const std::string& model_file)
{
	for (int i = 0; i < GetPoolSize(); ++i)
	{
		mNetPool[i]->LoadModel(model_file);
	}
}

void cNeuralNetTrainer::LoadScale(const std::string& scale_file)
{
	for (int i = 0; i < GetPoolSize(); ++i)
	{
		mNetPool[i]->LoadScale(scale_file);
	}
}

void cNeuralNetTrainer::Reset()
{
	ResetParams();
	BuildNets();

	if (EnableAsyncMode())
	{
		SyncNets();
	}

	ResetLearners();
}

void cNeuralNetTrainer::EndTraining()
{
	mDone = true;

#if defined(OUTPUT_TRAINER_LOG)
	EndLog();
	WriteLog(gLogFile);
#endif
}

void cNeuralNetTrainer::RequestLearner(std::shared_ptr<cNeuralNetLearner>& out_learner)
{
	out_learner = std::shared_ptr<cNeuralNetLearner>(new cNeuralNetLearner(shared_from_this()));
}

int cNeuralNetTrainer::RegisterLearner(cNeuralNetLearner* learner)
{
	int id = gInvalidIdx;
	auto iter = std::find(mLearners.begin(), mLearners.end(), learner);
	if (iter != mLearners.end())
	{
		assert(false); // learner already registered
	}
	else
	{
		id = static_cast<int>(mLearners.size());
		mLearners.push_back(learner);
	}
	return id;
}

void cNeuralNetTrainer::UnregisterLearner(cNeuralNetLearner* learner)
{
	auto iter = std::find(mLearners.begin(), mLearners.end(), learner);
	if (iter != mLearners.end())
	{
		size_t idx = iter - mLearners.begin();
		mLearners[idx] = mLearners[GetNumLearners() - 1];
		mLearners.pop_back();
	}
}

void cNeuralNetTrainer::SetIntOutputCallback(tCallbackFunc func)
{
	mIntOutputCallback = func;
}

bool cNeuralNetTrainer::EnableAsyncMode() const
{
	return mParamServer != nullptr;
}

void cNeuralNetTrainer::Lock()
{
#if defined(OUTPUT_TRAINER_LOG)
	TIMER_RECORD_BEG(LOCK_WAIT)
#endif
	mLock.lock();
#if defined(OUTPUT_TRAINER_LOG)
	TIMER_RECORD_END(LOCK_WAIT, mLog.mLockWaitTime, mLog.mLockWaitSamples)
#endif
}

void cNeuralNetTrainer::Unlock()
{
	mLock.unlock();
}

void cNeuralNetTrainer::SetParamServer(cParamServer* server)
{
	mParamServer = server;
}

int cNeuralNetTrainer::AddTuple(const tExpTuple& tuple, int prev_id, int learner_id)
{
	int id = mExpBuffer->AddTuple(tuple, prev_id, learner_id);
	bool valid_tuple = id != gInvalidIdx;

	if (mStage == eStageInit)
	{
		if (valid_tuple)
		{
			UpdateDataRecord(tuple);
		}
	}

	return id;
}

int cNeuralNetTrainer::AddTuples(const std::vector<tExpTuple>& tuples, int prev_id, int learner_id)
{
	int curr_prev_id = prev_id;
	for (size_t i = 0; i < tuples.size(); ++i)
	{
		curr_prev_id = AddTuple(tuples[i], curr_prev_id, learner_id);
	}
	return curr_prev_id;
}

int cNeuralNetTrainer::IncBufferHead(int head) const
{
	int buffer_size = GetPlaybackMemSize();
	int new_head = (head + 1) % buffer_size;
	return new_head;
}

void cNeuralNetTrainer::Train()
{
	UpdateStage();

	int num_init_samples = GetNumInitSamples();
	int num_tuples = mExpBuffer->GetNumTuples();
	if (mStage == eStageTrain && num_tuples >= num_init_samples)
	{
		ApplySteps(mParams.mNumStepsPerIter);
	}
}

const std::unique_ptr<cNeuralNet>& cNeuralNetTrainer::GetNet() const
{
	return GetCurrNet();
}

const std::unique_ptr<cNeuralNet>& cNeuralNetTrainer::GetCurrNet() const
{
	return mNetPool[mCurrActiveNet];
}

double cNeuralNetTrainer::GetDiscount() const
{
	double discount = (mParams.mRewardMode == eRewardModeAvg) ? 1 : mParams.mDiscount;
	return discount;
}

double cNeuralNetTrainer::GetAvgReward() const
{
	return mAvgReward;
}

int cNeuralNetTrainer::GetIter() const
{
	return mIter;
}

double cNeuralNetTrainer::NormalizeReward(double r) const
{
	double norm_r = r;

	switch (mParams.mRewardMode)
	{
	case eRewardModeStart:
		norm_r = r * CalcDiscountNorm(GetDiscount());
		break;
	case eRewardModeAvg:
		norm_r = r - GetAvgReward();
		break;
	default:
		assert(false); // unsupported reward mode
		break;
	}

	return norm_r;
}

void cNeuralNetTrainer::SetNumInitSamples(int num)
{
	mParams.mNumInitSamples = num;
}


void cNeuralNetTrainer::SetInputOffsetScaleType(const std::vector<cNeuralNet::eOffsetScaleType>& scale_types)
{
	assert(scale_types.size() == GetInputSize());
	mInputOffsetScaleTypes = scale_types;
}

void cNeuralNetTrainer::SetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	int offset_size = static_cast<int>(offset.size());
	int scale_size = static_cast<int>(scale.size());
	int num_inputs = GetInputSize();

	assert(offset_size == num_inputs);
	assert(scale_size == num_inputs);
	for (int i = 0; i < GetPoolSize(); ++i)
	{
		mNetPool[i]->SetInputOffsetScale(offset, scale);
	}
}

void cNeuralNetTrainer::SetOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	int offset_size = static_cast<int>(offset.size());
	int scale_size = static_cast<int>(scale.size());
	int num_outputs = GetOutputSize();

	assert(offset_size == num_outputs);
	assert(scale_size == num_outputs);
	for (int i = 0; i < GetPoolSize(); ++i)
	{
		mNetPool[i]->SetOutputOffsetScale(offset, scale);
	}
}

void cNeuralNetTrainer::GetInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	const auto& net = GetCurrNet();
	out_offset = net->GetInputOffset();
	out_scale = net->GetInputScale();
}

void cNeuralNetTrainer::GetOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	const auto& net = GetCurrNet();
	out_offset = net->GetOutputOffset();
	out_scale = net->GetOutputScale();
}

int cNeuralNetTrainer::GetNumInitSamples() const
{
	return mParams.mNumInitSamples;
}

const std::string& cNeuralNetTrainer::GetNetFile() const
{
	return mParams.mNetFile;
}

const std::string& cNeuralNetTrainer::GetSolverFile() const
{
	return mParams.mSolverFile;
}

cNeuralNetTrainer::eStage cNeuralNetTrainer::GetStage() const
{
	return mStage;
}

int cNeuralNetTrainer::GetNumTuples() const
{
	return mExpBuffer->GetTotalNumTuples();
}

void cNeuralNetTrainer::OutputModel(const std::string& filename) const
{
	const auto& curr_net = GetCurrNet();
	curr_net->OutputModel(filename);
}

bool cNeuralNetTrainer::HasInitModel() const
{
	bool has_init_model = false;
	const auto& curr_net = GetCurrNet();
	if (curr_net != nullptr)
	{
		has_init_model = curr_net->HasValidModel();
	}
	return has_init_model;
}

void cNeuralNetTrainer::EvalNet(const tExpTuple& tuple, Eigen::VectorXd& out_y)
{
	Eigen::VectorXd x;
	BuildTupleX(tuple, x);
	const auto& net = GetNet();
	net->Eval(x, out_y);
}

int cNeuralNetTrainer::GetStateSize() const
{
	int size = GetInputSize();
	return size;
}

int cNeuralNetTrainer::GetActionSize() const
{
	int size = GetOutputSize();
	return size;
}

int cNeuralNetTrainer::GetInputSize() const
{
	const auto& curr_net = GetCurrNet();
	return curr_net->GetInputSize();
}

int cNeuralNetTrainer::GetOutputSize() const
{
	const auto& curr_net = GetCurrNet();
	return curr_net->GetOutputSize();
}

int cNeuralNetTrainer::GetBatchSize() const
{
	const auto& curr_net = GetCurrNet();
	return curr_net->GetBatchSize();
}

int cNeuralNetTrainer::GetNumTuplesPerBatch() const
{
	return GetBatchSize();
}

void cNeuralNetTrainer::ResetExpBuffer()
{
	Lock();
	mExpBuffer->Reset();
	Unlock();
}

void cNeuralNetTrainer::BuildExpBuffer(std::unique_ptr<cExpBuffer>& out_exp_buffer) const
{
	out_exp_buffer = std::unique_ptr<cExpBuffer>(new cExpBuffer());
}

void cNeuralNetTrainer::InitExpBuffer(int buffer_size)
{
	cExpBuffer::tParams params;
	SetupExpBufferParams(buffer_size, params);
	mExpBuffer->Init(params);
}

void cNeuralNetTrainer::SetupExpBufferParams(int buffer_size, cExpBuffer::tParams& out_params) const
{
	out_params.mNumEntries = buffer_size;
	out_params.mStateBegSize = GetStateSize();
	out_params.mActionSize = GetActionSize();
	out_params.mStateEndSize = 0;
}

void cNeuralNetTrainer::InitInputOffsetScaleTypes()
{
	int input_size = GetInputSize();
	mInputOffsetScaleTypes.resize(input_size);
	for (int i = 0; i < input_size; ++i)
	{
		mInputOffsetScaleTypes[i] = cNeuralNet::eOffsetScaleTypeNone;
	}
}

void cNeuralNetTrainer::InitBatchBuffer()
{
	int batch_size = GetBatchSize();
	mBatchBuffer.resize(batch_size);
}

void cNeuralNetTrainer::InitProblem(cNeuralNet::tProblem& out_prob) const
{
	const auto& curr_net = GetCurrNet();
	const int x_size = GetProblemXSize();
	const int y_size = GetProblemYSize();
	int num_data = GetBatchSize();

	out_prob.mX.resize(num_data, x_size);
	out_prob.mY.resize(num_data, y_size);
	out_prob.mPassesPerStep = 1;
}

void cNeuralNetTrainer::InitDataRecord()
{
	int input_size = GetInputSize();
	mDataRecordX.Init(input_size);
}

int cNeuralNetTrainer::GetPlaybackMemSize() const
{
	return static_cast<int>(mExpBuffer->GetSize());
}

void cNeuralNetTrainer::ResetParams()
{
	if (mExpBuffer != nullptr)
	{
		mExpBuffer->Reset();
	}
	mCurrActiveNet = 0;
	mIter = 0;
	mStage = eStageInit;
}

int cNeuralNetTrainer::GetProblemXSize() const
{
	const auto& curr_net = GetCurrNet();
	return curr_net->GetInputSize();
}

int cNeuralNetTrainer::GetProblemYSize() const
{
	const auto& curr_net = GetCurrNet();
	return curr_net->GetOutputSize();
}

void cNeuralNetTrainer::Pretrain()
{
}

bool cNeuralNetTrainer::Step()
{
	bool succ = false;
	for (int i = 0; i < GetPoolSize(); ++i)
	{
		printf("Update Net %i:\n", i);
		succ = BuildProblem(i, mProb);
		if (succ)
		{
			UpdateNet(i, mProb);
		}
	}
	return succ;
}

bool cNeuralNetTrainer::BuildProblem(int net_id, cNeuralNet::tProblem& out_prob)
{
	bool succ = true;
	int num_data = GetNumTuplesPerBatch();
	FetchMinibatch(num_data, mBatchBuffer);

	if (mBatchBuffer.size() >= num_data)
	{
		{
#if defined(OUTPUT_TRAINER_LOG)
			TIMER_RECORD_BEG(BUILD_TUPLE_X)
#endif
			BuildProblemX(net_id, mBatchBuffer, out_prob);
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
			BuildProblemY(net_id, mBatchBuffer, out_prob.mX, out_prob);
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

void cNeuralNetTrainer::BuildProblemX(int net_id, const std::vector<int>& tuple_ids, cNeuralNet::tProblem& out_prob)
{
	int num_data = static_cast<int>(tuple_ids.size());
	assert(num_data == GetBatchSize());
	assert(out_prob.mX.rows() == num_data);
	
	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);

		Eigen::VectorXd x;
		BuildTupleX(tuple, x);
		out_prob.mX.row(i) = x;
	}
}

void cNeuralNetTrainer::BuildProblemY(int net_id, const std::vector<int>& tuple_ids, 
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
		BuildTupleY(net_id, tuple, y);
		out_prob.mY.row(i) = y;
	}
}

void cNeuralNetTrainer::UpdateMisc(const std::vector<int>& tuple_ids)
{
}

void cNeuralNetTrainer::UpdateDataRecord(const tExpTuple& tuple)
{
	Eigen::VectorXd x;
	BuildTupleX(tuple, x);
	mDataRecordX.Update(x);
}

void cNeuralNetTrainer::BuildTupleX(const tExpTuple& tuple, Eigen::VectorXd& out_x)
{
	out_x = tuple.mStateBeg;
}


void cNeuralNetTrainer::BuildTupleY(int net_id, const tExpTuple& tuple, Eigen::VectorXd& out_y)
{
	out_y = tuple.mAction;
}

void cNeuralNetTrainer::BuildTupleNextState(int net_id, const tExpTuple& tuple, Eigen::VectorXd& out_y)
{
	out_y = tuple.mStateEnd;
}

void cNeuralNetTrainer::FetchMinibatch(int size, std::vector<int>& out_batch)
{
	out_batch.resize(size);

	for (int i = 0; i < size; ++i)
	{
		int t = gInvalidIdx;
		if (mParams.mEnableExpReplay)
		{
			t = mExpBuffer->GetRandTupleID();
		}
		else
		{
			t = mExpBuffer->GetLastTupleID(i);
		}

		assert(t != gInvalidIdx);
		out_batch[i] = t;
	}
}

int cNeuralNetTrainer::GetTargetNetID(int net_id) const
{
	return net_id;
}

void cNeuralNetTrainer::UpdateCurrActiveNetID()
{
}

const std::unique_ptr<cNeuralNet>& cNeuralNetTrainer::GetTargetNet(int net_id) const
{
	int target_net_id = GetTargetNetID(net_id);
	return mNetPool[target_net_id];
}

void cNeuralNetTrainer::UpdateNet(int net_id, const cNeuralNet::tProblem& prob)
{
#if defined(OUTPUT_TRAINER_LOG)
	TIMER_RECORD_BEG(UPDATE_NET)
#endif
	auto& curr_net = mNetPool[net_id];
	if (EnableAsyncMode())
	{
#if defined(OUTPUT_TRAINER_LOG)
		TIMER_RECORD_BEG(ASYNC_FORWARD_BACKWARD)
#endif
		double loss = curr_net->ForwardBackward(prob);
		printf("Net %i Loss: %.8f\n", net_id, loss);

#if defined(OUTPUT_TRAINER_LOG)
		TIMER_RECORD_END(ASYNC_FORWARD_BACKWARD, mLog.mAsyncForwardBackTime, mLog.mAsyncForwardBackSamples)
#endif

#if defined(OUTPUT_TRAINER_LOG)
		TIMER_RECORD_BEG(ASYNC_UPDATE_NET)
#endif
		cParamServer::tInputInfo server_input;
		server_input.mID = net_id;
		server_input.mGradNet = curr_net.get();

		cParamServer::tOutputInfo server_output;
		server_output.mSyncNet = curr_net.get();

		mParamServer->UpdateNet(server_input, server_output);
		
#if defined(OUTPUT_TRAINER_LOG)
		TIMER_RECORD_END(ASYNC_UPDATE_NET, mLog.mAsyncUpdateNetTime, mLog.mAsyncUpdateNetSamples)
#endif

		if (net_id == 0)
		{
			mIter = server_output.mIter;
		}
	}
	else
	{
		curr_net->Train(prob);
	}
#if defined(OUTPUT_TRAINER_LOG)
	{
		std::lock_guard<std::mutex> lock(mLogLock);
		TIMER_RECORD_END(UPDATE_NET, mLog.mUpdateNetTime, mLog.mUpdateNetSamples)
	}
#endif
}

tExpTuple cNeuralNetTrainer::GetTuple(int t) const
{
	tExpTuple tuple = mExpBuffer->GetTuple(t);
	return tuple;
}

const Eigen::VectorXd& cNeuralNetTrainer::GetInputOffset() const
{
	const auto& curr_net = GetCurrNet();
	return curr_net->GetInputOffset();
}

const Eigen::VectorXd& cNeuralNetTrainer::GetInputScale() const
{
	const auto& curr_net = GetCurrNet();
	return curr_net->GetInputScale();
}

void cNeuralNetTrainer::UpdateOffsetScale()
{
	Eigen::VectorXd offset = GetInputOffset();
	Eigen::VectorXd scale = GetInputScale();
	mDataRecordX.CalcOffsetScale(mInputOffsetScaleTypes, mParams.mInputScaleMax, offset, scale);
	SetInputOffsetScale(offset, scale);

	if (EnableAsyncMode())
	{
		UpdateParamServerInputOffsetScale(offset, scale);
	}
}

void cNeuralNetTrainer::UpdateStage()
{
	if (mStage == eStageInit)
	{
		int num_init_samples = GetNumInitSamples();
		int num_tuples = mExpBuffer->GetNumTuples();
		if (num_tuples >= num_init_samples && num_tuples > 0)
		{
			InitStage();
			mStage = eStageTrain;
		}
	}
}

void cNeuralNetTrainer::InitStage()
{
	int batch_size = GetBatchSize();
	int num_init_samples = GetNumInitSamples();
	if (num_init_samples > 1 && mParams.mInitInputOffsetScale)
	{
		UpdateOffsetScale();
	}
	Pretrain();

#if defined(OUTPUT_TRAINER_LOG)
	InitLog();
#endif // OUTPUT_TRAINER_LOG
}

void cNeuralNetTrainer::ApplySteps(int num_steps)
{
	if (EnableIntOutput())
	{
		int curr_iter = GetIter();
		if (curr_iter % mParams.mIntOutputIters == 0)
		{
			if (mIntOutputCallback != nullptr)
			{
				mIntOutputCallback();
			}

			OutputIntermediate();
		}
	}
	
	bool succ_step = false;
	for (int i = 0; i < num_steps; ++i)
	{
#if defined(OUTPUT_TRAINER_LOG)
		TIMER_RECORD_BEG(TRAIN_STEP)
#endif
		succ_step = Step();
#if defined(OUTPUT_TRAINER_LOG)
		{
			std::lock_guard<std::mutex> lock(mLogLock);
			TIMER_RECORD_END(TRAIN_STEP, mLog.mStepTime, mLog.mStepSamples)
		}
#endif
	}

	if (succ_step)
	{
		IncIter();
		UpdateCurrActiveNetID();
	}
}

void cNeuralNetTrainer::IncIter()
{
	if (!EnableAsyncMode())
	{
		++mIter;
	}
}

int cNeuralNetTrainer::GetNetPoolSize() const
{
	return mParams.mPoolSize;
}

void cNeuralNetTrainer::BuildNets()
{
	int pool_size = GetPoolSize();
	BuildNetPool(mParams.mNetFile, mParams.mSolverFile, pool_size);
	LoadModels();
}

void cNeuralNetTrainer::BuildNetPool(const std::string& net_file, const std::string& solver_file,
								int pool_size)
{
	assert(pool_size > 0);
	pool_size = std::max(1, pool_size);
	mNetPool.clear();
	mNetPool.resize(pool_size);

	for (int i = 0; i < pool_size; ++i)
	{
		auto& net = mNetPool[i];
		net = std::unique_ptr<cNeuralNet>(new cNeuralNet());
		net->LoadNet(net_file);
		net->LoadSolver(solver_file);
	}
}

int cNeuralNetTrainer::GetPoolSize() const
{
	return mParams.mPoolSize;
}

void cNeuralNetTrainer::LoadModels()
{
	if (mParams.mModelFile != "")
	{
		LoadModel(mParams.mModelFile);
	}
}

bool cNeuralNetTrainer::EnableIntOutput() const
{
	return (mParams.mIntOutputFile != "") && (mParams.mIntOutputIters > 0);
}

void cNeuralNetTrainer::OutputIntermediate()
{
	int curr_iter = GetIter();
	std::string ext = cFileUtil::GetExtension(mParams.mIntOutputFile);
	std::string filename_noext = cFileUtil::RemoveExtension(mParams.mIntOutputFile);

	char str_buffer[128];
	sprintf(str_buffer, "%010d", curr_iter);
	std::string filename = filename_noext + "_" + str_buffer + "." + ext;
	OutputIntermediateModel(filename);

#if defined(OUTPUT_TRAINER_LOG)
	EndLog();
	WriteLog(gLogFile);
#endif
}

void cNeuralNetTrainer::OutputIntermediateModel(const std::string& filename) const
{
	OutputModel(filename);
}

int cNeuralNetTrainer::GetNumLearners() const
{
	return static_cast<int>(mLearners.size());
}

void cNeuralNetTrainer::ResetLearners()
{
	for (int i = 0; i < GetNumLearners(); ++i)
	{
		mLearners[i]->Reset();
	}
}

void cNeuralNetTrainer::ResetSolvers()
{
	for (int j = 0; j < GetNetPoolSize(); ++j)
	{
		if (EnableAsyncMode())
		{
			mParamServer->ResetSolver(j);
		}
		else
		{
			auto& curr_net = mNetPool[j];
			curr_net->ResetSolver();
		}
	}
}

void cNeuralNetTrainer::UpdateParamServerInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	assert(EnableAsyncMode());
	for (int i = 0; i < GetNetPoolSize(); ++i)
	{
		mParamServer->UpdateInputOffsetScale(i, offset, scale);
	}
}

void cNeuralNetTrainer::SyncNets()
{
	assert(EnableAsyncMode());
	for (int i = 0; i < GetNetPoolSize(); ++i)
	{
		SyncNet(i);
	}
}

bool cNeuralNetTrainer::IsDone() const
{
	return mDone;
}

void cNeuralNetTrainer::SyncNet(int net_id)
{
	auto& curr_net = mNetPool[net_id];
	mParamServer->SyncNet(net_id, *curr_net);
}

void cNeuralNetTrainer::OutputTuple(const tExpTuple& tuple, const std::string& out_file) const
{
	std::string tuple_str = "";
	tuple_str += "Reward: " + std::to_string(tuple.mReward) + "\n";
	
	tuple_str += "Start State: ";
	for (int i = 0; i < static_cast<int>(tuple.mStateBeg.size()); ++i)
	{
		double curr_val = tuple.mStateBeg[i];
		tuple_str += std::to_string(curr_val) + "\t";
	}
	tuple_str += "\n";

	tuple_str += "End State: ";
	for (int i = 0; i < static_cast<int>(tuple.mStateEnd.size()); ++i)
	{
		double curr_val = tuple.mStateEnd[i];
		tuple_str += std::to_string(curr_val) + "\t";
	}
	tuple_str += "\n";

	tuple_str += "Action: ";
	for (int i = 0; i < static_cast<int>(tuple.mAction.size()); ++i)
	{
		double curr_val = tuple.mAction[i];
		tuple_str += std::to_string(curr_val) + "\t";
	}
	tuple_str += "\n";

	cFileUtil::AppendText(tuple_str, out_file);
}

#if defined(OUTPUT_TRAINER_LOG)
cNeuralNetTrainer::tLog::tLog()
{
	mBuildTupleXSamples = 0;
	mBuildTupleXTime = 0;
	mBuildTupleYSamples = 0;
	mBuildTupleYTime = 0;
	mUpdateNetSamples = 0;
	mUpdateNetTime = 0;
	mStepSamples = 0;
	mStepTime = 0;

	mBuildActorTupleXSamples = 0;
	mBuildActorTupleXTime = 0;
	mBuildActorTupleYSamples = 0;
	mBuildActorTupleYTime = 0;
	mUpdateActorNetSamples = 0;
	mUpdateActorNetTime = 0;
	mStepActorSamples = 0;
	mStepActorTime = 0;

	mLockWaitSamples = 0;
	mLockWaitTime = 0;

	mTotalExpTime = 0;
	mTotalTime = 0;

	mAsyncForwardBackSamples = 0;
	mAsyncForwardBackTime = 0;
	mAsyncUpdateNetSamples = 0;
	mAsyncUpdateNetTime = 0;

	mIters = 0;
	mAvgIterTime = 0;
}

void cNeuralNetTrainer::tLog::Write(FILE* f) const
{
	fprintf(f, "Iterations: %i\n", mIters);
	fprintf(f, "Total Exp Time: %.10fs\n", mTotalExpTime);
	fprintf(f, "Total Time: %.5fs\n", mTotalTime);
	fprintf(f, "Avg Iter Time: %.5fs\n", mAvgIterTime);
	fprintf(f, "\n");

	fprintf(f, "Build Tuple X Time: %.10fs\n", mBuildTupleXTime);
	fprintf(f, "Build Tuple X Samples: %i\n", mBuildTupleXSamples);
	fprintf(f, "Build Tuple Y Time: %.10fs\n", mBuildTupleYTime);
	fprintf(f, "Build Tuple Y Samples: %i\n", mBuildTupleYSamples);
	fprintf(f, "Update Net Time: %.10fs\n", mUpdateNetTime);
	fprintf(f, "Update Net Samples: %i\n", mUpdateNetSamples);
	fprintf(f, "Step Time: %.10fs\n", mStepTime);
	fprintf(f, "Step Samples: %i\n", mStepSamples);
	fprintf(f, "\n");

	fprintf(f, "Build Actor Tuple X Time: %.10fs\n", mBuildActorTupleXTime);
	fprintf(f, "Build Actor Tuple X Samples: %i\n", mBuildActorTupleXSamples);
	fprintf(f, "Build Actor Tuple Y Time: %.10fs\n", mBuildActorTupleYTime);
	fprintf(f, "Build Actor Tuple Y Samples: %i\n", mBuildActorTupleYSamples);
	fprintf(f, "Update Actor Net Time: %.10fs\n", mUpdateActorNetTime);
	fprintf(f, "Update Actor Net Samples: %i\n", mUpdateActorNetSamples);
	fprintf(f, "Step Actor Time: %.10fs\n", mStepActorTime);
	fprintf(f, "Step Actor Samples: %i\n", mStepActorSamples);
	fprintf(f, "\n");

	fprintf(f, "Async Forward Backward Time: %.10fs\n", mAsyncForwardBackTime);
	fprintf(f, "Step Forward Backward Samples: %i\n", mAsyncForwardBackSamples);
	fprintf(f, "Async Update Net Time: %.10fs\n", mAsyncUpdateNetTime);
	fprintf(f, "Step Update Net Samples: %i\n", mAsyncUpdateNetSamples);
	fprintf(f, "\n");

	fprintf(f, "Lock Wait Time: %.10fs\n", mLockWaitTime);
	fprintf(f, "Lock Wait Samples: %i\n", mLockWaitSamples);
	fprintf(f, "\n");
}

const cNeuralNetTrainer::tLog& cNeuralNetTrainer::GetLog() const
{
	return mLog;
}

void cNeuralNetTrainer::InitLog()
{
	mLog = tLog();
	mStartTime = std::clock();
}

void cNeuralNetTrainer::EndLog()
{
	auto end_time = std::clock();
	mLog.mTotalTime = static_cast<double>(end_time - mStartTime) / CLOCKS_PER_SEC;
	mLog.mIters = GetIter();
	mLog.mAvgIterTime = mLog.mTotalTime / mLog.mIters;
}

void cNeuralNetTrainer::WriteLog(const std::string& log_file) const
{
	FILE* f = cFileUtil::OpenFile(gLogFile, "w");
	if (f != nullptr)
	{
		mLog.Write(f);
		cFileUtil::CloseFile(f);
	}
}

#endif // OUTPUT_TRAINER_LOG