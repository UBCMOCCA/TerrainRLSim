#include "QNetTrainer.h"
#include "util/FileUtil.h"

//#define FREEZE_TARGET_NET
#define CLAMP_VAL

const double cQNetTrainer::gValClampMin = 0;
const double cQNetTrainer::gValClampMax = 1;

cQNetTrainer::cQNetTrainer()
{
}

cQNetTrainer::~cQNetTrainer()
{
}

void cQNetTrainer::Init(const tParams& params)
{
	cNeuralNetTrainer::Init(params);
	InitBatchBuffers();
}

void cQNetTrainer::Clear()
{
	cNeuralNetTrainer::Clear();
	mBatchXBuffer.resize(0, 0);
	mBatchYBuffer0.resize(0, 0);
	mBatchYBuffer1.resize(0, 0);
}

void cQNetTrainer::BuildProblemY(int net_id, const std::vector<int>& tuple_ids, 
								const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob)
{
	int num_data = static_cast<int>(tuple_ids.size());
	int intput_size = GetInputSize();
	int output_size = GetOutputSize();
	assert(num_data == GetBatchSize());
	assert(out_prob.mY.rows() == num_data);
	assert(out_prob.mW.rows() == num_data);
	assert(out_prob.mY.cols() == out_prob.mW.cols());

	int ref_id = GetRandRefID(net_id);
	const auto& ref_net = mNetPool[ref_id];
	const auto& curr_net = mNetPool[net_id];

	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);
		mBatchXBuffer.row(i) = tuple.mStateEnd;
	}

	ref_net->EvalBatch(mBatchXBuffer, mBatchYBuffer0);
	curr_net->EvalBatch(mBatchXBuffer, mBatchYBuffer1);
	out_prob.mY.setZero();
	out_prob.mW.setZero();

	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);
		
		double new_q = 0;
		double r = tuple.mReward;

		double discount = GetDiscount();
		double norm = CalcDiscountNorm(discount);
		r *= norm;

		int action_idx = 0;
		tuple.mAction.maxCoeff(&action_idx);

		bool fail = tuple.GetFlag(tExpTuple::eFlagFail);
		if (fail)
		{
			new_q = r;
		}
		else
		{
			auto y_next_ref = mBatchYBuffer0.row(i);
			auto y_next = mBatchYBuffer1.row(i);

			int next_action_idx = 0;
			y_next.maxCoeff(&next_action_idx);

			double q_end = y_next_ref[next_action_idx];
#if defined(CLAMP_VAL)
			q_end = cMathUtil::Clamp(q_end, gValClampMin, gValClampMax);
#endif
			new_q = r + discount * q_end;
		}

		out_prob.mY(i, action_idx) = new_q;
		out_prob.mW(i, action_idx) = 1;
	}
}

void cQNetTrainer::BuildTupleY(int net_id, const tExpTuple& tuple, Eigen::VectorXd& out_y)
{
	double new_q = 0;
	double r = tuple.mReward;

	double discount = GetDiscount();
	double norm = CalcDiscountNorm(discount);
	r *= norm;
	
	int action_idx = 0;
	tuple.mAction.maxCoeff(&action_idx);

	const auto& curr_net = mNetPool[net_id];
	curr_net->Eval(tuple.mStateBeg, out_y);

	bool fail = tuple.GetFlag(tExpTuple::eFlagFail);
	if (fail)
	{
		new_q = r;
	}
	else
	{
		Eigen::VectorXd y_next;
		curr_net->Eval(tuple.mStateEnd, y_next);

		int next_action_idx = 0;
		y_next.maxCoeff(&next_action_idx);

		int ref_id = GetRandRefID(net_id);
		const auto& ref_net = mNetPool[ref_id];
		ref_net->Eval(tuple.mStateEnd, y_next);
		double q_end = y_next[next_action_idx];
#if defined(CLAMP_VAL)
		q_end = cMathUtil::Clamp(q_end, gValClampMin, gValClampMax);
#endif
		new_q = r + discount * q_end;
	}

#if defined (ENABLE_DEBUG_PRINT)
	double old_q = out_y[action_idx];
	//printf("Update action %i: old: %.5f, new: %.5f\n", action_idx, new_q, old_q);
#endif

	out_y[action_idx] = new_q;
}

void cQNetTrainer::InitBatchBuffers()
{
	int batch_size = GetBatchSize();
	if (batch_size > 0)
	{
		int input_size = GetInputSize();
		int output_size = GetOutputSize();
		mBatchXBuffer.resize(batch_size, input_size);
		mBatchYBuffer0.resize(batch_size, output_size);
		mBatchYBuffer1.resize(batch_size, output_size);
	}
}

void cQNetTrainer::InitProblem(cNeuralNet::tProblem& out_prob) const
{
	cNeuralNetTrainer::InitProblem(out_prob);
	out_prob.mW = Eigen::MatrixXd::Zero(out_prob.mY.rows(), out_prob.mY.cols());
}

void cQNetTrainer::SetupExpBufferParams(int buffer_size, cExpBuffer::tParams& out_params) const
{
	cNeuralNetTrainer::SetupExpBufferParams(buffer_size, out_params);
	out_params.mStateEndSize = GetStateSize();
}


bool cQNetTrainer::Step()
{
	int i = 0;
	int max_idx = GetPoolSize();

#if defined FREEZE_TARGET_NET
	i = mCurrActiveNet;
	max_idx = i + 1;
#endif // FREEZE_TARGET_NET

	for (i; i < max_idx; ++i)
	{
		printf("Update Net %i:\n", i);
		bool succ = BuildProblem(i, mProb);
		if (succ)
		{
			UpdateNet(i, mProb);
		}
	}

	return true;
}

void cQNetTrainer::UpdateCurrActiveNetID()
{
	mCurrActiveNet = GetNextActiveID();
}

int cQNetTrainer::GetNextActiveID() const
{
	int next_idx = 0;
	next_idx = (mCurrActiveNet + 1) % GetPoolSize();
#if defined FREEZE_TARGET_NET
	int iters = GetIter();
	if ((iters == 0) || (iters % mParams.mFreezeTargetIters != 0))
	{
		next_idx = mCurrActiveNet;
	}
#endif // FREEZE_TARGET_NET
	return next_idx;
}

int cQNetTrainer::GetRandRefID(int id) const
{
	int rand_id = 0;
	int pool_size = GetPoolSize();
	if (pool_size > 1)
	{
		rand_id = cMathUtil::RandIntExclude(0, pool_size, id);
	}
	return rand_id;
}