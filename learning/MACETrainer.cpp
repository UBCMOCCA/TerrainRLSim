#include "MACETrainer.h"
#include "util/FileUtil.h"
#include "util/Util.h"
#include "QNetTrainer.h"

//#define DISABLE_CRITIC_BUFFER
//#define DISABLE_ACTOR_BUFFER
#define CLAMP_VAL

int cMACETrainer::GetMaxFragIdx(const Eigen::VectorXd& params, int num_frags)
{
	int a = 0;
	params.segment(0, num_frags).maxCoeff(&a);
	return a;
}

double cMACETrainer::GetMaxFragVal(const Eigen::VectorXd& params, int num_frags)
{
	return params.segment(0, num_frags).maxCoeff();
}

void cMACETrainer::GetFrag(const Eigen::VectorXd& params, int num_frags, int frag_size, int a_idx, Eigen::VectorXd& out_params)
{
	out_params = params.segment(num_frags + a_idx * frag_size, frag_size);
}

void cMACETrainer::SetFrag(const Eigen::VectorXd& frag, int a_idx, int num_frags, int frag_size, Eigen::VectorXd& out_params)
{
	out_params.segment(num_frags + a_idx * frag_size, frag_size) = frag;
}

double cMACETrainer::GetVal(const Eigen::VectorXd& params, int a_idx)
{
	return params[a_idx];
}

void cMACETrainer::SetVal(double val, int a_idx, Eigen::VectorXd& out_params)
{
	out_params[a_idx] = val;
}

int cMACETrainer::CalcNumFrags(int param_size, int frag_size)
{
	return param_size / (frag_size + 1);
}

int cMACETrainer::GetActionFragIdx(const Eigen::VectorXd& action_params)
{
	int a_idx = static_cast<int>(action_params[0]);
	return a_idx;
}

void cMACETrainer::SetActionFragIdx(int a_idx, Eigen::VectorXd& out_action_params)
{
	out_action_params[0] = a_idx;
}

void cMACETrainer::GetActionFrag(const Eigen::VectorXd& action_params, Eigen::VectorXd& out_frag_params)
{
	out_frag_params = action_params.segment(1, action_params.size() - 1);
}

void cMACETrainer::SetActionFrag(const Eigen::VectorXd& frag_params, Eigen::VectorXd& out_action_params)
{
	out_action_params.segment(1, out_action_params.size() - 1) = frag_params;
}


cMACETrainer::cMACETrainer()
{
	mNumActionFrags = 1;
	mActionFragSize = 1;
}

cMACETrainer::~cMACETrainer()
{
}

void cMACETrainer::Init(const tParams& params)
{
	assert(params.mPoolSize == 1); // different pool sizes not yet supported

	mActorIter = 0;
	mActorBuffer.clear();
	mCriticBuffer.clear();
	mActorBatchBuffer.clear();

	cNeuralNetTrainer::Init(params);
	InitBatchBuffers();
	InitActorProblem(mActorProb);
}

void cMACETrainer::Reset()
{
	mActorIter = 0;
	mActorBuffer.clear();
	mCriticBuffer.clear();
	mActorBatchBuffer.clear();

	cNeuralNetTrainer::Reset();
}

int cMACETrainer::AddTuple(const tExpTuple& tuple, int prev_id, int learner_id)
{
	int t = cNeuralNetTrainer::AddTuple(tuple, prev_id, learner_id);
	UpdateBuffers(t);
	return t;
}

void cMACETrainer::SetNumActionFrags(int num)
{
	mNumActionFrags = num;
}

void cMACETrainer::SetActionFragSize(int size)
{
	mActionFragSize = size;
}

int cMACETrainer::GetNumActionFrags() const
{
	return mNumActionFrags;
}

int cMACETrainer::GetActionFragSize() const
{
	return mActionFragSize;
}

const std::unique_ptr<cNeuralNet>& cMACETrainer::GetActor() const
{
	return GetCurrNet();
}

void cMACETrainer::InitBatchBuffers()
{
	int batch_size = GetBatchSize();
	if (batch_size > 0)
	{
		int input_size = GetInputSize();
		int output_size = GetOutputSize();
		mBatchXBuffer.resize(batch_size, input_size);
		mBatchYBuffer.resize(batch_size, output_size);
		mBatchValBuffer0.resize(batch_size);
		mBatchValBuffer1.resize(batch_size);
	}
}

void cMACETrainer::InitProblem(cNeuralNet::tProblem& out_prob) const
{
	cNeuralNetTrainer::InitProblem(out_prob);
	out_prob.mW = Eigen::MatrixXd::Zero(out_prob.mY.rows(), out_prob.mY.cols());
}

void cMACETrainer::InitActorProblem(cNeuralNet::tProblem& out_prob) const
{
	const auto& net = GetActor();
	const int x_size = net->GetInputSize();
	const int y_size = net->GetOutputSize();
	int num_data = GetActorBatchSize();

	out_prob.mX.resize(num_data, x_size);
	out_prob.mY.resize(num_data, y_size);
	out_prob.mPassesPerStep = 1;
	out_prob.mW = Eigen::MatrixXd::Zero(num_data, y_size);
}

void cMACETrainer::SetupExpBufferParams(int buffer_size, cExpBuffer::tParams& out_params) const
{
	cNeuralNetTrainer::SetupExpBufferParams(buffer_size, out_params);
	out_params.mStateEndSize = GetStateSize();
}

void cMACETrainer::FetchMinibatch(int size, std::vector<int>& out_batch)
{
#if defined(DISABLE_CRITIC_BUFFER)
	cNeuralNetTrainer::FetchMinibatch(size, out_batch);
#else
	int critic_buffer_size = static_cast<int>(mCriticBuffer.size());
	if (critic_buffer_size >= size)
	{
		out_batch.resize(size);
		for (int i = 0; i < size; ++i)
		{
			int idx = cMathUtil::RandInt(0, critic_buffer_size);
			int t = mCriticBuffer[idx];
			assert(!IsExpActor(t));

			out_batch[i] = t;
		}
	}
	else
	{
		out_batch.clear();
	}
#endif
}

void cMACETrainer::FetchActorMinibatch(int batch_size, std::vector<int>& out_batch)
{
	int num_exp_actor = static_cast<int>(mActorBuffer.size());
	int num_samples = std::min(batch_size, num_exp_actor);
	out_batch.clear();
	out_batch.reserve(num_samples);

	for (int i = 0; i < num_samples; ++i)
	{
		int rand_idx = cMathUtil::RandInt(0, num_exp_actor);
		int t = mActorBuffer[rand_idx];

#if defined(DISABLE_ACTOR_BUFFER)
		t = cMathUtil::RandInt(0, mNumTuples);
#endif
		bool contains = (std::find(mActorBatchBuffer.begin(), mActorBatchBuffer.end(), t) != mActorBatchBuffer.end())
						|| (std::find(out_batch.begin(), out_batch.end(), t) != out_batch.end());
		if (!contains)
		{
			out_batch.push_back(t);
		}
	}
}

void cMACETrainer::BuildNetPool(const std::string& net_file, const std::string& solver_file, int pool_size)
{
	cNeuralNetTrainer::BuildNetPool(net_file, solver_file, pool_size);
	UpdateTargetNet();
}

void cMACETrainer::BuildProblemY(int net_id, const std::vector<int>& tuple_ids,
								const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob)
{
	int num_data = static_cast<int>(tuple_ids.size());
	assert(num_data == GetBatchSize());
	assert(out_prob.mY.rows() == num_data);

	CalcNewCumulativeRewardBatch(net_id, tuple_ids, mBatchValBuffer0);
	out_prob.mY.setZero();
	out_prob.mW.setZero();

	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);

		double new_q = mBatchValBuffer0(i);

		int a = GetActionFragIdx(tuple.mAction);

		Eigen::VectorXd curr_y = out_prob.mY.row(i);
		Eigen::VectorXd curr_w = out_prob.mW.row(i);
		SetValAux(new_q, a, curr_y);
		SetValAux(1, a, curr_w);

		out_prob.mY.row(i) = curr_y;
		out_prob.mW.row(i) = curr_w;
	}
}

void cMACETrainer::BuildTupleActorY(const tExpTuple& tuple, Eigen::VectorXd& out_y)
{
	out_y = Eigen::VectorXd::Zero(GetActorOutputSize());

	int a = GetActionFragIdx(tuple.mAction);
	Eigen::VectorXd frag;
	GetActionFrag(tuple.mAction, frag);
	SetFragAux(frag, a, out_y);
}

void cMACETrainer::BuildActorProblemX(int num_data, const std::vector<int>& tuple_ids, cNeuralNet::tProblem& out_prob)
{
	assert(num_data <= GetBatchSize());
	assert(num_data <= out_prob.mX.rows());

	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);

		Eigen::VectorXd x;
		BuildTupleX(tuple, x);
		out_prob.mX.row(i) = x;
	}
}

void cMACETrainer::BuildActorProblemY(int num_data, const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob)
{
	out_prob.mY.setZero();
	out_prob.mW.setZero();

	const Eigen::VectorXd frag_w = Eigen::VectorXd::Ones(GetActionFragSize());
	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);

		int a = GetActionFragIdx(tuple.mAction);
		Eigen::VectorXd frag;
		GetActionFrag(tuple.mAction, frag);

		Eigen::VectorXd curr_y = out_prob.mY.row(i);
		Eigen::VectorXd curr_w = out_prob.mW.row(i);
		SetFragAux(frag, a, curr_y);
		SetFragAux(frag_w, a, curr_w);

		out_prob.mY.row(i) = curr_y;
		out_prob.mW.row(i) = curr_w;
	}
}

int cMACETrainer::GetActorBatchSize() const
{
	const auto& actor = GetActor();
	return actor->GetBatchSize();
}

void cMACETrainer::BuildTupleY(int net_id, const tExpTuple& tuple, Eigen::VectorXd& out_y)
{
	double new_q = CalcNewCumulativeReward(net_id, tuple);
	out_y = Eigen::VectorXd::Zero(GetCriticOutputSize());
	int a = GetActionFragIdx(tuple.mAction);
	SetValAux(new_q, a, out_y);
}

void cMACETrainer::ApplySteps(int num_steps)
{
	int critic_buffer_count = static_cast<int>(mCriticBuffer.size());
	int actor_buffer_count = static_cast<int>(mActorBuffer.size());

	printf("Critic Buffer Count %i\n", critic_buffer_count);
	printf("Actor Buffer Count %i\n", actor_buffer_count);
	printf("Actor Iter %i\n", mActorIter);
	cNeuralNetTrainer::ApplySteps(num_steps);
}

int cMACETrainer::GetPoolSize() const
{
	int pool_size = GetNetPoolSize();
	if (EnableTargetNet())
	{
		pool_size *= 2;
	}
	return pool_size;
}

bool cMACETrainer::Step()
{
	bool succ = false;
	for (int i = 0; i < GetNetPoolSize(); ++i)
	{
		printf("Update Net %i:\n", i);
		succ = BuildProblem(i, mProb);
		if (succ)
		{
			UpdateNet(i, mProb);
		}
	}

	UpdateActor();

	if (EnableTargetNet())
	{
		int iters = GetIter();
		if ((iters > 0) && (iters % mParams.mFreezeTargetIters == 0))
		{
			UpdateTargetNet();
		}
	}

	return succ;
}

int cMACETrainer::GetTargetNetID(int net_id) const
{
	int target_id = net_id;
	if (EnableTargetNet())
	{
		target_id = mParams.mPoolSize + net_id;
	}
	return target_id;
}

double cMACETrainer::CalcCurrCumulativeReward(int net_id, const tExpTuple& tuple)
{
	const auto& tar_net = GetTargetNet(net_id);

	Eigen::VectorXd x;
	BuildTupleX(tuple, x);

	Eigen::VectorXd y;
	tar_net->Eval(x, y);
	double val = GetMaxFragValAux(y);

	return val;
}

double cMACETrainer::CalcNewCumulativeReward(int net_id, const tExpTuple& tuple)
{
	double new_q = 0;
	double r = tuple.mReward;

	double discount = GetDiscount();
	double norm = CalcDiscountNorm(discount);
	r *= norm;

	const auto& tar_net = GetTargetNet(net_id);

	bool fail = tuple.GetFlag(tExpTuple::eFlagFail);
	if (fail)
	{
		new_q = r;
	}
	else
	{
		Eigen::VectorXd y_next;
		tar_net->Eval(tuple.mStateEnd, y_next);
		double q_end = GetMaxFragValAux(y_next);
#if defined(CLAMP_VAL)
		q_end = cMathUtil::Clamp(q_end, cQNetTrainer::gValClampMin, cQNetTrainer::gValClampMax);
#endif
		new_q = r + discount * q_end;
	}

	return new_q;
}

void cMACETrainer::CalcCurrCumulativeRewardBatch(int net_id, const std::vector<int>& tuple_ids,
												Eigen::VectorXd& out_vals)
{
	const int num_data = static_cast<int>(tuple_ids.size());
	assert(num_data <= GetBatchSize());
	const auto& tar_net = GetTargetNet(net_id);

	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);
		Eigen::VectorXd x;
		BuildTupleX(tuple, x);
		mBatchXBuffer.row(i) = x;
	}

	tar_net->EvalBatch(mBatchXBuffer, mBatchYBuffer);

	for (int i = 0; i < num_data; ++i)
	{
		auto curr_y = mBatchYBuffer.row(i);
		double val = GetMaxFragValAux(curr_y);
		out_vals(i) = val;
	}
}

void cMACETrainer::CalcNewCumulativeRewardBatch(int net_id, const std::vector<int>& tuple_ids,
												Eigen::VectorXd& out_vals)
{
	const int num_data = static_cast<int>(tuple_ids.size());
	assert(num_data <= GetBatchSize());
	const auto& tar_net = GetTargetNet(net_id);

	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);
		mBatchXBuffer.row(i) = tuple.mStateEnd;
	}

	tar_net->EvalBatch(mBatchXBuffer, mBatchYBuffer);

	double discount = GetDiscount();
	double norm = CalcDiscountNorm(discount);

	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);

		double new_q = 0;
		double r = tuple.mReward;
		r *= norm;

		bool fail = tuple.GetFlag(tExpTuple::eFlagFail);
		if (fail)
		{
			new_q = r;
		}
		else
		{
			auto y_next = mBatchYBuffer.row(i);
			double q_end = GetMaxFragValAux(y_next);
#if defined(CLAMP_VAL)
			q_end = cMathUtil::Clamp(q_end, cQNetTrainer::gValClampMin, cQNetTrainer::gValClampMax);
#endif
			new_q = r + discount * q_end;
		}
		out_vals(i) = new_q;
	}
}

void cMACETrainer::UpdateActorBatchBuffer()
{
	int batch_size = GetActorBatchSize();
	FetchActorMinibatch(batch_size, mBatchBuffer);
	int num_samples = static_cast<int>(mBatchBuffer.size());

	int net_id = mCurrActiveNet;
	CalcCurrCumulativeRewardBatch(net_id, mBatchBuffer, mBatchValBuffer0);
	CalcNewCumulativeRewardBatch(net_id, mBatchBuffer, mBatchValBuffer1);

	for (int i = 0; i < num_samples; ++i)
	{
		double curr_val = mBatchValBuffer0[i];
		double new_val = mBatchValBuffer1[i];

		if (new_val > curr_val)
		{
			int t = mBatchBuffer[i];
			mActorBatchBuffer.push_back(t);
		}
	}
}

void cMACETrainer::UpdateActor()
{
#if defined(ENABLE_ACTOR_MULTI_SAMPLE_UPDATE)
	if (mStage != eStageInit)
	{
		UpdateActorBatchBuffer();
	}
#endif

	int batch_size = GetBatchSize();
	int buffer_size = static_cast<int>(mActorBatchBuffer.size());
	int num_batches = buffer_size / batch_size;

	for (int b = 0; b < num_batches; ++b)
	{
		StepActor();
		mActorBatchBuffer.erase(mActorBatchBuffer.begin(), mActorBatchBuffer.begin() + batch_size);
	}
}

void cMACETrainer::StepActor()
{
#if defined(OUTPUT_TRAINER_LOG)
	TIMER_RECORD_BEG(TRAIN_STEP_ACTOR)
#endif

	BuildActorProblem(mActorProb);
	UpdateActorNet(mActorProb);
	IncActorIter();

#if defined(OUTPUT_TRAINER_LOG)
	{
		std::lock_guard<std::mutex> lock(mLogLock);
		TIMER_RECORD_BEG(TRAIN_STEP_ACTOR, mLog.mStepActorTime, mLog.mStepActorSamples)
	}
#endif
}

void cMACETrainer::BuildActorProblem(cNeuralNet::tProblem& out_prob)
{
	int num_data = GetActorBatchSize();

	const auto& actor = GetActor();
	const int x_size = actor->GetInputSize();
	const int y_size = actor->GetOutputSize();

	int buffer_size = static_cast<int>(mActorBatchBuffer.size());
	assert(buffer_size >= num_data);

#if defined(OUTPUT_TRAINER_LOG)
	TIMER_RECORD_BEG(BUILD_ACTOR_TUPLE_X)
#endif
	BuildActorProblemX(num_data, mActorBatchBuffer, out_prob);
#if defined(OUTPUT_TRAINER_LOG)
	{
		std::lock_guard<std::mutex> lock(mLogLock);
		TIMER_RECORD_END(BUILD_ACTOR_TUPLE_X, mLog.mBuildActorTupleXTime, mLog.mBuildActorTupleXSamples)
	}
#endif

#if defined(OUTPUT_TRAINER_LOG)
	TIMER_RECORD_BEG(BUILD_ACTOR_TUPLE_Y)
#endif
	BuildActorProblemY(num_data, mActorBatchBuffer, out_prob.mX, out_prob);
#if defined(OUTPUT_TRAINER_LOG)
	{
		std::lock_guard<std::mutex> lock(mLogLock);
		TIMER_RECORD_END(BUILD_ACTOR_TUPLE_Y, mLog.mBuildActorTupleYTime, mLog.mBuildActorTupleYSamples)
	}
#endif
}

void cMACETrainer::UpdateActorNet(const cNeuralNet::tProblem& prob)
{
	int net_id = mCurrActiveNet;

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
		printf("Actor Net Loss: %.8f\n", net_id, loss);

#if defined(OUTPUT_TRAINER_LOG)
		TIMER_RECORD_END(ASYNC_FORWARD_BACKWARD, mLog.mAsyncForwardBackTime, mLog.mAsyncForwardBackSamples)
#endif

#if defined(OUTPUT_TRAINER_LOG)
		TIMER_RECORD_BEG(ASYNC_UPDATE_NET)
#endif
		cParamServer::tInputInfo server_input;
		server_input.mID = net_id;
		server_input.mGradNet = curr_net.get();
		server_input.mIncIter = false;

		cParamServer::tOutputInfo server_output;
		server_output.mSyncNet = curr_net.get();

		mParamServer->UpdateNet(server_input, server_output);
		
#if defined(OUTPUT_TRAINER_LOG)
		TIMER_RECORD_END(ASYNC_UPDATE_NET, mLog.mAsyncUpdateNetTime, mLog.mAsyncUpdateNetSamples)
#endif
		
		++mActorIter;
	}
	else
	{
		curr_net->Train(prob);
	}

#if defined(OUTPUT_TRAINER_LOG)
	{
		std::lock_guard<std::mutex> lock(mLogLock);
		TIMER_RECORD_END(UPDATE_NET, mLog.mUpdateActorNetTime, mLog.mUpdateActorNetSamples)
	}
#endif
}


void cMACETrainer::IncActorIter()
{
	++mActorIter;
}


void cMACETrainer::UpdateBuffers(int t)
{
	if (t != gInvalidIdx)
	{
		bool exp_actor = IsExpActor(t);

#if defined(ENABLE_ACTOR_MULTI_SAMPLE_UPDATE)
		{
			auto off_beg = mActorBuffer.begin();
			auto off_end = mActorBuffer.end();
			auto off_iter = std::find(off_beg, off_end, t);

			bool off_contains = off_iter != off_end;
			if (exp_actor)
			{
				if (!off_contains)
				{
					mActorBuffer.push_back(t);
				}
			}
			else if (off_contains)
			{
				int idx = static_cast<int>(off_iter - off_beg);
				int last_val = mActorBuffer[mActorBuffer.size() - 1];
				mActorBuffer[idx] = last_val;
				mActorBuffer.pop_back();
			}
		}
#endif

#if !defined(DISABLE_CRITIC_BUFFER)
		{
			auto critic_beg = mCriticBuffer.begin();
			auto critic_end = mCriticBuffer.end();
			auto critic_iter = std::find(critic_beg, critic_end, t);

			bool critic_contains = critic_iter != critic_end;
			if (!exp_actor)
			{
				if (!critic_contains)
				{
					mCriticBuffer.push_back(t);
				}
			}
			else if (critic_contains)
			{
				int idx = static_cast<int>(critic_iter - critic_beg);
				int last_val = mCriticBuffer[mCriticBuffer.size() - 1];
				mCriticBuffer[idx] = last_val;
				mCriticBuffer.pop_back();
			}
		}
#endif
		{
			auto actor_buffer_iter = std::find(mActorBatchBuffer.begin(), mActorBatchBuffer.end(), t);
			while (actor_buffer_iter != mActorBatchBuffer.end())
			{
				int idx = static_cast<int>(actor_buffer_iter - mActorBatchBuffer.begin());
				int last_val = mActorBatchBuffer[mActorBatchBuffer.size() - 1];
				mActorBatchBuffer[idx] = last_val;
				mActorBatchBuffer.pop_back();

				actor_buffer_iter = std::find(mActorBatchBuffer.begin(), mActorBatchBuffer.end(), t);
			}
		}
	}
}

bool cMACETrainer::IsExpCritic(int t) const
{
	int flag = mExpBuffer->GetFlags(t);
	bool off_policy = tExpTuple::TestFlag(flag, tExpTuple::eFlagExpCritic);
	return off_policy;
}

bool cMACETrainer::IsExpActor(int t) const
{
	int flag = mExpBuffer->GetFlags(t);
	bool explore = tExpTuple::TestFlag(flag, tExpTuple::eFlagExpActor);
	return explore;
}


bool cMACETrainer::EnableTargetNet() const
{
	bool enable = (mParams.mFreezeTargetIters > 0);
	return enable;
}

void cMACETrainer::UpdateTargetNet()
{
	if (EnableTargetNet())
	{
		for (int i = 0; i < GetNetPoolSize(); ++i)
		{
			auto& net = mNetPool[i];
			auto& target_net = GetTargetNet(i);
			target_net->CopyModel(*net.get());
		}
	}
}


int cMACETrainer::GetMaxFragIdxAux(const Eigen::VectorXd& params)
{
	return cMACETrainer::GetMaxFragIdx(params, mNumActionFrags);
}

double cMACETrainer::GetMaxFragValAux(const Eigen::VectorXd& params)
{
	return cMACETrainer::GetMaxFragVal(params, mNumActionFrags);
}

void cMACETrainer::GetFragAux(const Eigen::VectorXd& params, int a_idx, Eigen::VectorXd& out_action)
{
	cMACETrainer::GetFrag(params, mNumActionFrags, mActionFragSize, a_idx, out_action);
}

void cMACETrainer::SetFragAux(const Eigen::VectorXd& frag, int a_idx, Eigen::VectorXd& out_params)
{
	cMACETrainer::SetFrag(frag, a_idx, mNumActionFrags, mActionFragSize, out_params);
}

double cMACETrainer::GetValAux(const Eigen::VectorXd& params, int a_idx)
{
	return cMACETrainer::GetVal(params, a_idx);
}

void cMACETrainer::SetValAux(double val, int a_idx, Eigen::VectorXd& out_params)
{
	cMACETrainer::SetVal(val, a_idx, out_params);
}