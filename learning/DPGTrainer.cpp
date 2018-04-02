#include "DPGTrainer.h"

//#define ENABLE_REG_GRAD
#define ENABLE_INVERT_GRAD
//#define ENABLE_BOUND_GRAD

cDPGTrainer::cDPGTrainer()
{
	mQDiff = 1;
	mTargetLerp = 0.001;
	mDPGReg = 0.01;
}

cDPGTrainer::~cDPGTrainer()
{
}

void cDPGTrainer::SetQDiff(double q_diff)
{
	mQDiff = q_diff;
}

void cDPGTrainer::SetDPGReg(double reg)
{
	mDPGReg = reg;
}

void cDPGTrainer::FetchActorMinibatch(int batch_size, std::vector<int>& out_batch)
{
	cNeuralNetTrainer::FetchMinibatch(batch_size, out_batch);
}

void cDPGTrainer::BuildTupleActorY(const tExpTuple& tuple, Eigen::VectorXd& out_y)
{
	const auto& actor_net = GetActor();

	Eigen::VectorXd actor_x;
	Eigen::VectorXd actor_y;
	BuildTupleActorX(tuple, actor_x);
	actor_net->Eval(actor_x, actor_y);
	
	Eigen::VectorXd dpg;
	tExpTuple dpg_tuple = tuple;
	dpg_tuple.mAction = actor_y;
	CalcDPG(dpg_tuple, dpg);

	printf("DPG:\n");
	for (int i = 0; i < dpg.size(); ++i)
	{
		printf("%.8f\t", dpg[i]);
	}
	printf("\n");

	actor_y += dpg;
	out_y = actor_y;
}


void cDPGTrainer::CalcDPG(const tExpTuple& tuple, Eigen::VectorXd& out_dpg)
{
	CalcDPGIntern(tuple, out_dpg);
	ProcessPoliGrad(tuple.mAction, out_dpg);
}

void cDPGTrainer::CalcDPGIntern(const tExpTuple& tuple, Eigen::VectorXd& out_dpg)
{
	int state_size = GetStateSize();
	int action_size = GetActionSize();
	out_dpg = Eigen::VectorXd::Zero(action_size);

	Eigen::VectorXd x;
	Eigen::VectorXd y;
	Eigen::VectorXd curr_dpg;
	BuildTupleX(tuple, x);
	curr_dpg.resize(x.size());

	double q_diff = GetQDiff();
	int num_critics = GetNetPoolSize();

	for (int i = 0; i < num_critics; ++i)
	{
		const auto& critic_net = mNetPool[i];
		critic_net->Eval(x, y);
		y = q_diff * Eigen::VectorXd::Ones(y.size());
		critic_net->Backward(y, curr_dpg);

		out_dpg += curr_dpg.segment(state_size, action_size);
	}
	out_dpg /= num_critics;
}

void cDPGTrainer::CalcDPGNum(const tExpTuple& tuple, double delta, Eigen::VectorXd& out_dpg)
{
	int state_size = GetStateSize();
	int action_size = GetActionSize();
	out_dpg.resize(action_size);

	Eigen::VectorXd x;
	Eigen::VectorXd y;
	BuildTupleX(tuple, x);

	const auto& critic = GetCritic();
	for (int i = 0; i < action_size; ++i)
	{
		int idx = state_size + i;
		double prev_val = x[idx];
		x[idx] = prev_val - delta;

		critic->Eval(x, y);
		double val0 = y[0];

		x[idx] = prev_val + delta;
		critic->Eval(x, y);
		double val1 = y[0];

		double diff = (val1 - val0) / (2 * delta);
		out_dpg[i] = diff;
		x[idx] = prev_val;
	}

	double q_diff = GetQDiff();
	out_dpg *= q_diff;
}

double cDPGTrainer::CheckDPG(const tExpTuple& tuple, double delta)
{
	Eigen::VectorXd dpg;
	Eigen::VectorXd n_dpg;
	CalcDPGIntern(tuple, dpg);
	CalcDPGNum(tuple, delta, n_dpg);

	Eigen::VectorXd diff = n_dpg - dpg;
	double err = diff.lpNorm<Eigen::Infinity>();
	return err;
}

double cDPGTrainer::GetQDiff() const
{
	return mQDiff;
}

void cDPGTrainer::UpdateActor()
{
	StepActor();
}

void cDPGTrainer::BuildActorProblem(cNeuralNet::tProblem& out_prob)
{
	FetchActorMinibatch(GetActorBatchSize(), mActorBatchBuffer);
	BuildActorProblemX(mActorBatchBuffer, out_prob);
	BuildActorProblemY(mActorBatchBuffer, out_prob.mX, out_prob);
}

void cDPGTrainer::BuildActorProblemY(const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob)
{
	const auto& actor_net = GetActor();
	actor_net->EvalBatch(X, out_prob.mY);

	int num_tups = static_cast<int>(tuple_ids.size());
	for (int i = 0; i < num_tups; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);
		Eigen::VectorXd curr_action = out_prob.mY.row(i);
		
		Eigen::VectorXd dpg;
		tuple.mAction = curr_action;
		CalcDPG(tuple, dpg);

		printf("DPG:\n");
		for (int j = 0; j < dpg.size(); ++j)
		{
			printf("%.6f\t", dpg[j]);
		}
		printf("\n");

		out_prob.mY.row(i) += dpg;
	}
}

void cDPGTrainer::UpdateBuffers(int t)
{
}

void cDPGTrainer::ProcessPoliGrad(const Eigen::VectorXd& action, Eigen::VectorXd& out_dpg) const
{
	int action_size = GetActionSize();
	assert(action.size() == action_size);
	assert(out_dpg.size() == action_size);

	Eigen::VectorXd action_scale;
	GetActionScale(action_scale);
	action_scale = action_scale.cwiseProduct(action_scale);
	out_dpg = out_dpg.cwiseQuotient(action_scale);

#if defined(ENABLE_REG_GRAD)
	Eigen::VectorXd reg_action;
	GetRegAction(reg_action);

	Eigen::VectorXd delta = reg_action - action;
	delta *= mDPGReg;
	out_dpg += delta;
#endif

	for (int i = 0; i < action_size; ++i)
	{
		double dpg_val = out_dpg[i];
		double action_val = action[i];

		double bound_min = mActionMin[i];
		double bound_max = mActionMax[i];

#if defined(ENABLE_INVERT_GRAD)
		double dpg_scale = 1;
		if (dpg_val > 0 && std::isfinite(bound_max))
		{
			dpg_scale = (bound_max - action_val) / (bound_max - bound_min);
		}
		else if (dpg_val < 0 && std::isfinite(bound_min))
		{
			dpg_scale = (action_val - bound_min) / (bound_max - bound_min);
		}
		dpg_val *= dpg_scale;
#endif

#if defined(ENABLE_BOUND_GRAD)
		if (dpg_val >= 0)
		{
			if (std::isfinite(bound_max))
			{
				if (dpg_val + action_val > bound_max)
				{
					dpg_val = std::max(0.0, bound_max - action_val);
				}
			}
		}
		else
		{
			if (std::isfinite(bound_min))
			{
				if (dpg_val + action_val < bound_min)
				{
					dpg_val = std::min(0.0, bound_min - action_val);
				}
			}
		}
#endif
		out_dpg[i] = dpg_val;
	}
}

void cDPGTrainer::GetRegAction(Eigen::VectorXd& out_action) const
{
	const auto& actor = GetActor();
	out_action = actor->GetOutputOffset();
	out_action = -out_action;
}

void cDPGTrainer::GetActionScale(Eigen::VectorXd& out_scale) const
{
	const auto& actor = GetActor();
	out_scale = actor->GetOutputScale();
}