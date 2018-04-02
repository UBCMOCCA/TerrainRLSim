#include "BaseControllerMACEDPG.h"
#include "learning/MACEDPGTrainer.h"

//#define DISABLE_INIT_ACTOR_BIAS
//#define ENABLE_ACTOR_BIAS_NOISE

cBaseControllerMACEDPG::cBaseControllerMACEDPG() : cBaseControllerDPG()
{
	mNumActionFrags = 0;
	mExpCritic = false;
	mExpActor = false;
}

cBaseControllerMACEDPG::~cBaseControllerMACEDPG()
{
}

void cBaseControllerMACEDPG::Reset()
{
	cBaseControllerDPG::Reset();
	mExpCritic = false;
	mExpActor = false;
}


int cBaseControllerMACEDPG::GetNumActionFrags() const
{
	return mNumActionFrags;
}

int cBaseControllerMACEDPG::GetActionFragSize() const
{
	return GetPoliActionSize();
}

int cBaseControllerMACEDPG::GetNetOutputSize() const
{
	return GetActorOutputSize();
}

int cBaseControllerMACEDPG::GetActorOutputSize() const
{
	return GetNumActionFrags() * GetActionFragSize();
}

int cBaseControllerMACEDPG::GetCriticInputSize() const
{
	return GetPoliStateSize() + GetPoliActionSize();
}

void cBaseControllerMACEDPG::BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	Eigen::VectorXd action_frag_offset;
	Eigen::VectorXd action_frag_scale;
	BuildActionFragOutputOffsetScale(action_frag_offset, action_frag_scale);

	int output_size = GetActorOutputSize();
	int num_frags = GetNumActionFrags();
	assert(output_size == num_frags * action_frag_offset.size());

	out_offset = Eigen::VectorXd::Zero(output_size);
	out_scale = Eigen::VectorXd::Ones(output_size);

	for (int f = 0; f < num_frags; ++f)
	{
		Eigen::VectorXd curr_frag_offset;
		BuildActorBias(f, curr_frag_offset);
		curr_frag_offset = -curr_frag_offset;

#if defined(DISABLE_INIT_ACTOR_BIAS)
		curr_frag_offset = action_frag_offset;
#endif

#if defined(ENABLE_ACTOR_BIAS_NOISE)
		const double noise_scale = 0.5;
		for (int i = 0; i < static_cast<int>(curr_frag_offset.size()); ++i)
		{
			double curr_scale = 1.0 / action_frag_scale[i];
			double rand_noise = cMathUtil::RandDoubleNorm(0, noise_scale);
			curr_frag_offset[i] += curr_scale * rand_noise;
		}
#endif

		SetFrag(curr_frag_offset, f, out_offset);
		SetFrag(action_frag_scale, f, out_scale);
	}
}

void cBaseControllerMACEDPG::BuildActionFragOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int action_frag_size = GetActionFragSize();
	int num_actions = GetNumActions();

	if (num_actions > 0)
	{
		int default_action_id = GetDefaultAction();
		if (default_action_id == gInvalidIdx)
		{
			default_action_id = 0;
		}

		out_offset = Eigen::VectorXd::Zero(action_frag_size);
		out_scale = Eigen::VectorXd::Ones(action_frag_size);

		BuildActionOptParams(default_action_id, out_offset);
		out_offset *= -1;

		if (num_actions > 1)
		{
			out_scale.setZero();
			Eigen::VectorXd param_buffer;
			for (int a = 0; a < num_actions; ++a)
			{
				if (a != default_action_id)
				{
					BuildActionOptParams(a, param_buffer);
					param_buffer += out_offset;
					param_buffer = param_buffer.cwiseAbs();
					out_scale = out_scale.cwiseMax(param_buffer);
				}
			}

			out_scale = out_scale.cwiseInverse();
		}
	}
}

void cBaseControllerMACEDPG::LoadNetIntern(const std::string& net_file)
{
	cBaseControllerDPG::LoadNetIntern(net_file);
	UpdateFragParams();
}

void cBaseControllerMACEDPG::UpdateFragParams()
{
	int num_outputs = mNet->GetOutputSize();
	mNumActionFrags = cMACEDPGTrainer::CalcNumFrags(num_outputs, GetActionFragSize());
	mBoltzmannBuffer.resize(mNumActionFrags);
}

void cBaseControllerMACEDPG::ProcessCommand(tAction& out_action)
{
	mExpActor = true;
	mExpCritic = true;
}

void cBaseControllerMACEDPG::CalcCriticVals(const Eigen::VectorXd& state, const Eigen::VectorXd& actions, Eigen::VectorXd& out_vals)
{
	cMACEDPGTrainer::CalcCriticVals(GetCritic().get(), state, actions, out_vals);
}

void cBaseControllerMACEDPG::BuildActorAction(const Eigen::VectorXd& actions, int a_id, tAction& out_action) const
{
	Eigen::VectorXd action_frag;
	GetFrag(actions, a_id, action_frag);
	assert(action_frag.size() == GetNumOptParams());

	out_action.mID = a_id;
	out_action.mParams = mCurrAction.mParams;
	SetOptParams(action_frag, out_action.mParams);
}

void cBaseControllerMACEDPG::UpdateAction()
{
	mExpCritic = false;
	mExpActor = false;
}

void cBaseControllerMACEDPG::DecideAction(tAction& out_action)
{
	DecideActionBoltzmann(out_action);
}

void cBaseControllerMACEDPG::ExploitPolicy(const Eigen::VectorXd& state, tAction& out_action)
{
	Eigen::VectorXd actions;
	mNet->Eval(state, actions);

	CalcCriticVals(state, actions, mBoltzmannBuffer);

	int a = GetMaxValIdx(mBoltzmannBuffer);
	double val = GetVal(mBoltzmannBuffer, a);
	BuildActorAction(actions, a, out_action);

#if defined(ENABLE_DEBUG_VISUALIZATION)
	mPoliValLog.Add(val);
	mVisNNOutput = mBoltzmannBuffer;
#endif // ENABLE_DEBUG_VISUALIZATION

#if defined (ENABLE_DEBUG_PRINT)
	PrintCriticVals(mBoltzmannBuffer, a);
#endif
}

void cBaseControllerMACEDPG::DecideActionBoltzmann(tAction& out_action)
{
	mIsOffPolicy = false;
	double base_rand = cMathUtil::RandDouble();
	if (mEnableExp && base_rand < mExpParams.mBaseActionRate)
	{
		BuildRandBaseAction(out_action);
		mIsOffPolicy = true;
		mExpActor = true;
		mExpCritic = true;
	}
	else
	{
		const auto& actor = GetActor();

		Eigen::VectorXd actions;
		actor->Eval(mPoliState, actions);
		CalcCriticVals(mPoliState, actions, mBoltzmannBuffer);

#if defined(ENABLE_DEBUG_VISUALIZATION)
		Eigen::VectorXd critic_y = mBoltzmannBuffer;
#endif // ENABLE_DEBUG_VISUALIZATION

		int a_max = GetMaxValIdx(mBoltzmannBuffer);
		int a = a_max;

		if (mEnableExp)
		{
			a = BoltzmannSelectActor(mBoltzmannBuffer, mBoltzmannBuffer);
		}

		double val = GetVal(mBoltzmannBuffer, a);
		BuildActorAction(actions, a, out_action);

		if (mEnableExp)
		{
			double rand_noise = cMathUtil::RandDouble();
			if (rand_noise < mExpParams.mRate)
			{
				ApplyExpNoise(out_action);
				mExpActor = true;
			}

			mExpCritic = (a != a_max);
			mIsOffPolicy = mExpActor || mExpCritic;
		}


#if defined(ENABLE_DEBUG_VISUALIZATION)
		mPoliValLog.Add(val);
		mVisNNOutput = critic_y;
#endif // ENABLE_DEBUG_VISUALIZATION

#if defined (ENABLE_DEBUG_PRINT) && defined(ENABLE_DEBUG_VISUALIZATION)
		if (mExpActor || mExpCritic)
		{
			printf("\n");
			if (mExpActor)
			{
				printf("Actor ");
			}
			if (mExpCritic)
			{
				printf("Critic ");
			}
			printf("Exploration\n");
		}

		PrintCriticVals(critic_y, a);
#endif
	}
}

int cBaseControllerMACEDPG::BoltzmannSelectActor(const Eigen::VectorXd& vals, Eigen::VectorXd& val_buffer) const
{
	int a_max = GetMaxValIdx(vals);
	int a = a_max;

	if (mExpParams.mTemp != 0)
	{
		int num_actors = GetNumActionFrags();
		double max_val = GetVal(vals, a_max);

		double sum = 0;
		for (int i = 0; i < num_actors; ++i)
		{
			double curr_val = GetVal(vals, i);
			curr_val = std::exp((curr_val - max_val) / mExpParams.mTemp);

			val_buffer[i] = curr_val;
			sum += curr_val;
		}

		double rand = cMathUtil::RandDouble(0, sum);
		for (int i = 0; i < num_actors; ++i)
		{
			double curr_val = val_buffer[i];
			rand -= curr_val;

			if (rand <= 0)
			{
				a = i;
				break;
			}
		}

#if defined (ENABLE_DEBUG_PRINT)
		printf("Boltzmann:\t");
		for (int i = 0; i < num_actors; ++i)
		{
			double curr_val = val_buffer[i];
			curr_val /= sum;
			printf("%.3f\t", curr_val);
		}
		printf("\n");
#endif
	}

	return a;
}


int cBaseControllerMACEDPG::GetMaxValIdx(const Eigen::VectorXd& vals) const
{
	return cMACEDPGTrainer::GetMaxFragValIdx(vals);
}

double cBaseControllerMACEDPG::GetMaxVal(const Eigen::VectorXd& vals) const
{
	return cMACEDPGTrainer::GetMaxFragVal(vals);
}

void cBaseControllerMACEDPG::GetFrag(const Eigen::VectorXd& actions, int a_idx, Eigen::VectorXd& out_action) const
{
	cMACEDPGTrainer::GetFrag(actions, GetActionFragSize(), a_idx, out_action);
}

void cBaseControllerMACEDPG::SetFrag(const Eigen::VectorXd& action, int a_idx, Eigen::VectorXd& out_actions) const
{
	cMACEDPGTrainer::SetFrag(action, a_idx, GetActionFragSize(), out_actions);
}

double cBaseControllerMACEDPG::GetVal(const Eigen::VectorXd& vals, int a_idx) const
{
	return cMACEDPGTrainer::GetVal(vals, a_idx);
}

void cBaseControllerMACEDPG::SetVal(double val, int a_idx, Eigen::VectorXd& out_vals) const
{
	cMACEDPGTrainer::SetVal(val, a_idx, out_vals);
}


#if defined (ENABLE_DEBUG_PRINT)
void cBaseControllerMACEDPG::PrintCriticVals(const Eigen::VectorXd& vals, int a_id) const
{
	double val = GetVal(vals, a_id);
	printf("Val: (%.3f)\t", val);
	for (int f = 0; f < vals.size(); ++f)
	{
		double curr_val = GetVal(vals, f);
		printf("%.3f\t", curr_val);
	}
	printf("\n");
}
#endif


#if defined(ENABLE_DEBUG_VISUALIZATION)
void cBaseControllerMACEDPG::GetVisActionFeatures(Eigen::VectorXd& out_features) const
{
	BuildOptParams(out_features);
	if (HasNet())
	{
		Eigen::VectorXd offset;
		Eigen::VectorXd scale;
		// use the same offset and scale for all actors for easier visual comparisons
		//int a_id = 0;
		int a_id = GetCurrActionID();
		GetFrag(mNet->GetOutputOffset(), a_id, offset);
		GetFrag(mNet->GetOutputScale(), a_id, scale);
		out_features = scale.cwiseProduct(out_features + offset);
	}
}

void cBaseControllerMACEDPG::GetVisActionValues(Eigen::VectorXd& out_vals) const
{
	if (mVisNNOutput.size() > 0)
	{
		out_vals = mVisNNOutput.segment(0, GetNumActionFrags());
	}
	else
	{
		out_vals.resize(0);
	}
}
#endif