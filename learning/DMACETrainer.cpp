#include "DMACETrainer.h"
#include "CaclaTrainer.h"
#include "MACETrainer.h"
#include "DMACELearner.h"

#define ENABLE_NEG_CACLA
#define DISABLE_TEMP_TD_SCALE

cDMACETrainer::cDMACETrainer()
{
	mNumActionFrags = 1;
	mActionFragSize = 1;
	mTemp = 1;
	mGateScale = 1;
	mValBound = 1;
}

cDMACETrainer::~cDMACETrainer()
{
}

void cDMACETrainer::Init(const tParams& params)
{
	mParams = params;
	cCaclaTrainer::Init(params);
	InitActorProblem(mActorProb);
}

void cDMACETrainer::SetNumActionFrags(int num)
{
	mNumActionFrags = num;
}

void cDMACETrainer::SetActionFragSize(int size)
{
	mActionFragSize = size;
}

int cDMACETrainer::GetNumActionFrags() const
{
	return mNumActionFrags;
}

int cDMACETrainer::GetActionFragSize() const
{
	return mActionFragSize;
}

int cDMACETrainer::GetActionSize() const
{
	int size = 1 + mActionFragSize;
	return size;
}

void cDMACETrainer::SetValBound(double bound)
{
	mValBound = bound;
}

void cDMACETrainer::SetTemp(double temp)
{
	mTemp = temp;
}

void cDMACETrainer::SetGateScale(double scale)
{
	mGateScale = scale;
}

void cDMACETrainer::RequestLearner(std::shared_ptr<cNeuralNetLearner>& out_learner)
{
	out_learner = std::shared_ptr<cDMACELearner>(new cDMACELearner(shared_from_this()));
}

void cDMACETrainer::BuildActorProblemY(const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob)
{
	const auto& net = GetActor();
	net->EvalBatch(X, out_prob.mY);

	int num_actors = mNumActionFrags;
	int batch_size = GetActorBatchSize();
	
	for (int i = 0; i < batch_size; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);

		double td = 1;
		ePGMode mode = GetPGMode();
		if (mode == ePGModeTD || mode == ePGModePTD)
		{
			td = mActorBatchTDBuffer[i];
			printf("TD: %.5f\n", td);
			td *= mParams.mPGAdvScale;
		}
		else if (mode == ePGModeCacla)
		{
#if defined(ENABLE_NEG_CACLA)
			td = mActorBatchTDBuffer[i];
			td = (td >= 0) ? 1 : -1;
#endif
		}

		Eigen::VectorXd frag;
		cMACETrainer::GetActionFrag(tuple.mAction, frag);

		Eigen::VectorXd curr_y = out_prob.mY.row(i);

		printf("Vals: ");
		for (int a = 0; a < num_actors; ++a)
		{
			printf("%.3f\t", curr_y[a]);
		}
		printf("\n");
		
		int selected_a = cMACETrainer::GetActionFragIdx(tuple.mAction);
		int max_a = GetMaxFragIdxAux(curr_y);

		if (mode == ePGModeTD || mode == ePGModePTD)
		{
			Eigen::VectorXd prev_frag;
			GetFragAux(curr_y, selected_a, prev_frag);

			Eigen::VectorXd diff = frag - prev_frag;
			diff *= td;
			
			ProcessPoliGrad(prev_frag, td, diff);
			frag = prev_frag + diff;
		}
		else if (mode == ePGModeCacla)
		{
#if defined(ENABLE_NEG_CACLA)
			if (td < 0)
			{
				GetFragAux(curr_y, selected_a, frag);
			}
#endif
		}

		SetFragAux(frag, selected_a, curr_y);

		Eigen::VectorXd softmax(num_actors);
		cMathUtil::CalcSoftmax(curr_y.segment(0, num_actors), mTemp, softmax);
		double sm_scale = 1.0 / mTemp;
#if defined(DISABLE_TEMP_TD_SCALE)
		sm_scale = 1;
#endif
		sm_scale *= td;
		sm_scale *= mGateScale;

		softmax *= -sm_scale;
		softmax[selected_a] += sm_scale;

		ProcessPoliGradGate(curr_y, softmax);
		curr_y.segment(0, num_actors) += softmax;

		for (int a = 0; a < num_actors; ++a)
		{
			double val = cMACETrainer::GetVal(curr_y, a);
			val = cMathUtil::Clamp(val, -mValBound, mValBound);
			cMACETrainer::SetVal(val, a, curr_y);
		}
		
		out_prob.mY.row(i) = curr_y;

		printf("Deltas: ");
		for (int a = 0; a < num_actors; ++a)
		{
			printf("%.3f\t", softmax[a]);
		}
		printf("\n");
	}
}

bool cDMACETrainer::IsOffPolicy(int t) const
{
	int flag = mExpBuffer->GetFlags(t);
	bool off_policy = tExpTuple::TestFlag(flag, tExpTuple::eFlagExpActor);
	return off_policy;
}

int cDMACETrainer::GetMaxFragIdxAux(const Eigen::VectorXd& params)
{
	return cMACETrainer::GetMaxFragIdx(params, mNumActionFrags);
}

double cDMACETrainer::GetMaxFragValAux(const Eigen::VectorXd& params)
{
	return cMACETrainer::GetMaxFragVal(params, mNumActionFrags);
}

void cDMACETrainer::GetFragAux(const Eigen::VectorXd& params, int a_idx, Eigen::VectorXd& out_action)
{
	cMACETrainer::GetFrag(params, mNumActionFrags, mActionFragSize, a_idx, out_action);
}

void cDMACETrainer::SetFragAux(const Eigen::VectorXd& frag, int a_idx, Eigen::VectorXd& out_params)
{
	cMACETrainer::SetFrag(frag, a_idx, mNumActionFrags, mActionFragSize, out_params);
}

double cDMACETrainer::GetValAux(const Eigen::VectorXd& params, int a_idx)
{
	return cMACETrainer::GetVal(params, a_idx);
}

void cDMACETrainer::SetValAux(double val, int a_idx, Eigen::VectorXd& out_params)
{
	cMACETrainer::SetVal(val, a_idx, out_params);
}

bool cDMACETrainer::CheckActorTD(double td) const
{
#if defined(ENABLE_NEG_CACLA)
	return true;
#else
	return cCaclaTrainer::CheckActorTD(td);
#endif
}

void cDMACETrainer::ProcessPoliGradGate(const Eigen::VectorXd& val, Eigen::VectorXd& out_diff) const
{
	int diff_size = static_cast<int>(out_diff.size());
	assert(diff_size == mNumActionFrags);

	double bound_min = -mValBound;
	double bound_max = mValBound;
	for (int i = 0; i < diff_size; ++i)
	{
		double curr_diff = out_diff[i];
		double curr_val = val[i];
		/*
		double scale = 1;
		if (curr_diff >= 0)
		{
			scale = (bound_max - curr_val) / (bound_max - bound_min);
		}
		else
		{
			scale = (curr_val - bound_min) / (bound_max - bound_min);
		}
		curr_diff *= scale;
		*/
		if (curr_diff >= 0)
		{
			if (curr_val + curr_diff > bound_max)
			{
				curr_diff = std::max(0.0, bound_max - curr_val);
			}
		}
		else
		{
			if (curr_val + curr_diff < bound_min)
			{
				curr_diff = std::min(0.0, bound_min - curr_val);
			}
		}
		
		out_diff[i] = curr_diff;
	}
}