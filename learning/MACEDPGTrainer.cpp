#include "MACEDPGTrainer.h"
#include "util/FileUtil.h"
#include "QNetTrainer.h"

#define CLAMP_VAL

void cMACEDPGTrainer::CalcCriticVals(const cNeuralNet* critic_net, const Eigen::VectorXd& state, const Eigen::VectorXd& actions,
									Eigen::VectorXd& out_vals)
{
	int input_size = critic_net->GetInputSize();
	int state_size = static_cast<int>(state.size());
	int action_size = input_size - state_size;

	Eigen::VectorXd state_action = Eigen::VectorXd(input_size);
	state_action.segment(0, state_size) = state;

	int num_actions = static_cast<int>(actions.size()) / action_size;
	Eigen::VectorXd critic_output;
	for (int a = 0; a < num_actions; ++a)
	{
		state_action.segment(state_size, action_size) = actions.segment(a * action_size, action_size);
		critic_net->Eval(state_action, critic_output);
		double val = critic_output[0];
		out_vals[a] = val;
	}
}

int cMACEDPGTrainer::GetMaxFragValIdx(const Eigen::VectorXd& vals)
{
	int a = 0;
	vals.maxCoeff(&a);
	return a;
}

double cMACEDPGTrainer::GetMaxFragVal(const Eigen::VectorXd& vals)
{
	int a = GetMaxFragValIdx(vals);
	return GetVal(vals, a);
}

void cMACEDPGTrainer::GetFrag(const Eigen::VectorXd& actions, int frag_size, int a_idx, Eigen::VectorXd& out_action)
{
	out_action = actions.segment(a_idx * frag_size, frag_size);
}

void cMACEDPGTrainer::SetFrag(const Eigen::VectorXd& action, int a_idx, int frag_size, Eigen::VectorXd& out_actions)
{
	out_actions.segment(a_idx * frag_size, frag_size) = action;
}

double cMACEDPGTrainer::GetVal(const Eigen::VectorXd& vals, int a_idx)
{
	return vals[a_idx];
}

void cMACEDPGTrainer::SetVal(double val, int a_idx, Eigen::VectorXd& out_vals)
{
	out_vals[a_idx] = val;
}

int cMACEDPGTrainer::CalcNumFrags(int param_size, int frag_size)
{
	return param_size / frag_size;
}

cMACEDPGTrainer::cMACEDPGTrainer()
{
	mNumActionFrags = 1;
	mActionFragSize = 1;
	mTemp = 1;
}

cMACEDPGTrainer::~cMACEDPGTrainer()
{
}

void cMACEDPGTrainer::Clear()
{
	cDPGTrainer::Clear();
	mBatchCriticValBuffer.resize(0, 0);
	mBatchDPGBuffer.resize(0, 0);
}

void cMACEDPGTrainer::SetNumActionFrags(int num)
{
	mNumActionFrags = num;
}

void cMACEDPGTrainer::SetActionFragSize(int size)
{
	mActionFragSize = size;
}

void cMACEDPGTrainer::SetTemp(double temp)
{
	mTemp = temp;
}

void cMACEDPGTrainer::RequestLearner(std::shared_ptr<cNeuralNetLearner>& out_learner)
{
	out_learner = std::shared_ptr<cMACEDPGLearner>(new cMACEDPGLearner(shared_from_this()));
}

void cMACEDPGTrainer::InitBatchBuffers()
{
	cDPGTrainer::InitBatchBuffers();

	int batch_size = GetBatchSize();
	int num_actors = mNumActionFrags;
	int action_size = GetActionSize();
	mBatchCriticValBuffer.resize(batch_size, num_actors);
	mBatchDPGBuffer.resize(batch_size, num_actors * action_size);
}

int cMACEDPGTrainer::GetActionSize() const
{
	int size = mActionFragSize;
	return size;
}

void cMACEDPGTrainer::CalcNewCumulativeRewardBatch(int net_id, const std::vector<int>& tuple_ids,
													Eigen::VectorXd& out_vals)
{
	const int num_data = static_cast<int>(tuple_ids.size());
	assert(num_data <= GetBatchSize());
	const auto& critic = GetTargetNet(net_id);
	const auto& actor = GetActorTarget();

	int state_size = GetStateSize();
	int action_size = GetActionSize();
	Eigen::MatrixXd& X_next = mBatchXBuffer;

	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);

		Eigen::VectorXd curr_x_next;
		BuildActorTupleXNext(tuple, curr_x_next);
		mBatchActorXBuffer.row(i) = curr_x_next;

		X_next.row(i).segment(0, state_size) = curr_x_next;
	}
	
	actor->EvalBatch(mBatchActorXBuffer, mBatchActorYBuffer);
	mBatchValBuffer0.setZero();

	for (int a = 0; a < mNumActionFrags; ++a)
	{
		X_next.block(0, state_size, num_data, action_size) = mBatchActorYBuffer.block(0, a * action_size, num_data, action_size);
		critic->EvalBatch(X_next, mBatchYBuffer);
		mBatchValBuffer0 = mBatchValBuffer0.cwiseMax(mBatchYBuffer);
	}

	double discount = GetDiscount();
	double norm = CalcDiscountNorm(discount);

	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);

		double val = 0;
		double r = tuple.mReward;
		r *= norm;

		bool fail = tuple.GetFlag(tExpTuple::eFlagFail);
		if (fail)
		{
			val = r;
		}
		else
		{
			double v_end = mBatchValBuffer0(i);
#if defined(CLAMP_VAL)
			v_end = cMathUtil::Clamp(v_end, cQNetTrainer::gValClampMin, cQNetTrainer::gValClampMax);
#endif
			val = r + discount * v_end;
		}

		out_vals(i) = val;
	}
}

void cMACEDPGTrainer::BuildActorProblemY(const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob)
{
	const auto& critic_net = GetCritic();
	const auto& actor_net = GetActor();
	const auto& critic_tar = GetCriticTarget();

	int state_size = GetStateSize();
	int action_size = GetActionSize();
	int num_actors = mNumActionFrags;
	double q_diff = GetQDiff();

	actor_net->EvalBatch(X, out_prob.mY);

	int num_tups = static_cast<int>(tuple_ids.size());
	for (int i = 0; i < num_tups; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);
		const auto actions = out_prob.mY.row(i);

		Eigen::VectorXd x;
		Eigen::VectorXd y;
		Eigen::VectorXd x_dpg;
		x.resize(state_size + action_size);
		x.segment(0, state_size) = tuple.mStateBeg;
		x_dpg.resize(x.size());

		for (int a = 0; a < num_actors; ++a)
		{
			Eigen::VectorXd curr_action;
			GetFrag(actions, action_size, a, curr_action);

			x.segment(state_size, action_size) = curr_action;
			critic_tar->Eval(x, y);
			double curr_val = y[0];

			critic_net->Eval(x, y);
			y = q_diff * Eigen::VectorXd::Ones(y.size());
			critic_net->Backward(y, x_dpg);

			Eigen::VectorXd dpg = x_dpg.segment(state_size, action_size);
			ProcessPoliGrad(curr_action, a, dpg);

			mBatchCriticValBuffer(i, a) = curr_val;
			mBatchDPGBuffer.block(i, a * action_size, 1, action_size) = dpg.transpose();
		}
	}

	double temp = mTemp;
	for (int i = 0; i < num_tups; ++i)
	{
		auto curr_vals = mBatchCriticValBuffer.row(i);
		auto curr_dpgs = mBatchDPGBuffer.row(i);
		double max_val = GetMaxFragVal(curr_vals);

		double E = 0;
		double sum = 0;
		for (int a = 0; a < num_actors; ++a)
		{
			double val = curr_vals[a];
			// - max_val for more stable calculation
			double exp_val = std::exp((val - max_val) / temp);
			E += val * exp_val;
			sum += exp_val;
		}
		E /= sum;

		double max_mult = 0;
		for (int a = 0; a < num_actors; ++a)
		{
			double val = curr_vals[a];
			double exp_val = std::exp((val - max_val) / temp);
			double p = exp_val / sum;
			double dpg_mult = p * (1 + 1 / temp * (val - E));
			dpg_mult = std::max(0.0, dpg_mult);

			curr_vals[a] = dpg_mult;
			max_mult = std::max(max_mult, dpg_mult);
		}

		curr_vals /= max_mult;
		for (int a = 0; a < num_actors; ++a)
		{
			double dpg_mult = curr_vals[a];
			curr_dpgs.segment(a * action_size, action_size) *= dpg_mult;
		}

		printf("DPG: ");
		for (int a = 0; a < num_actors; ++a)
		{
			double dpg_mult = curr_vals[a];
			printf("(%.4f)\t", dpg_mult);
		}
		printf("\n");

		for (int j = 0; j < curr_dpgs.size(); ++j)
		{
			printf("%.6f\t", curr_dpgs[j]);
		}
		printf("\n");
	}

	out_prob.mY += mBatchDPGBuffer;
}

void cMACEDPGTrainer::ProcessPoliGrad(const Eigen::VectorXd& action, int actor_id, Eigen::VectorXd& out_dpg) const
{
	int action_size = GetActionSize();
	assert(action.size() == action_size);
	assert(out_dpg.size() == action_size);
	
	Eigen::VectorXd reg_action;
	GetRegAction(actor_id, reg_action);

	Eigen::VectorXd action_scale;
	GetActionScale(actor_id, action_scale);
	action_scale = action_scale.cwiseProduct(action_scale);
	out_dpg = out_dpg.cwiseQuotient(action_scale);

	Eigen::VectorXd delta = reg_action - action;
	delta *= mDPGReg;

	out_dpg += delta;
}

void cMACEDPGTrainer::GetRegAction(int actor_id, Eigen::VectorXd& out_action) const
{
	const auto& actor = GetActor();
	Eigen::VectorXd offset = actor->GetOutputOffset();
	GetFrag(offset, GetActionSize(), actor_id, out_action);
	out_action = -out_action;
}

void cMACEDPGTrainer::GetActionScale(int actor_id, Eigen::VectorXd& out_scale) const
{
	const auto& actor = GetActor();
	Eigen::VectorXd scale = actor->GetOutputScale();
	GetFrag(scale, GetActionSize(), actor_id, out_scale);
}
