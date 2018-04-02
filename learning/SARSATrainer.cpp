#include "learning/SARSATrainer.h"
#include "learning/QNetTrainer.h"

//#define ENABLE_LERP_TARGET
#define CLAMP_VAL

cSARSATrainer::cSARSATrainer()
{
}

cSARSATrainer::~cSARSATrainer()
{
}

void cSARSATrainer::Clear()
{
	cCaclaTrainer::Clear();

	mBatchActorXBuffer.resize(0, 0);
	mBatchActorYBuffer.resize(0, 0);

	mActorTargetNet.reset();
	mActionMin.resize(0);
	mActionMax.resize(0);
}

void cSARSATrainer::SetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	SetActorInputOffsetScale(offset, scale);
}

void cSARSATrainer::SetActorInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	cCaclaTrainer::SetActorInputOffsetScale(offset, scale);
	if (EnableTargetNet())
	{
		const auto& actor_tar = GetActorTarget();
		actor_tar->SetInputOffsetScale(offset, scale);
	}
}

void cSARSATrainer::SetActorOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	cCaclaTrainer::SetActorOutputOffsetScale(offset, scale);
	if (EnableTargetNet())
	{
		const auto& actor_target = GetActorTarget();
		actor_target->SetOutputOffsetScale(offset, scale);
	}
}

void cSARSATrainer::SetTargetLerp(double lerp)
{
	mTargetLerp = lerp;
}

void cSARSATrainer::LoadActorModel(const std::string& model_file)
{
	cCaclaTrainer::LoadActorModel(model_file);
	if (EnableTargetNet())
	{
		const auto& actor_target = GetActorTarget();
		actor_target->LoadModel(model_file);
	}
}

void cSARSATrainer::LoadActorScale(const std::string& scale_file)
{
	cCaclaTrainer::LoadActorScale(scale_file);
	if (EnableTargetNet())
	{
		const auto& actor_target = GetActorTarget();
		actor_target->LoadScale(scale_file);
	}
}

void cSARSATrainer::BuildActor(const std::string& net_file, const std::string& solver_file)
{
	cCaclaTrainer::BuildActor(net_file, solver_file);
	if (EnableTargetNet())
	{
		mActorTargetNet = std::unique_ptr<cNeuralNet>(new cNeuralNet());
		mActorTargetNet->LoadNet(net_file);
		mActorTargetNet->LoadSolver(solver_file);

		const auto& actor = GetActor();
		mActorTargetNet->CopyModel(*actor.get());
	}
}

const std::unique_ptr<cNeuralNet>& cSARSATrainer::GetActorTarget() const
{
	if (EnableTargetNet())
	{
		return mActorTargetNet;
	}
	return GetActor();
}

void cSARSATrainer::InitBatchBuffers()
{
	cCaclaTrainer::InitBatchBuffers();

	int batch_size = GetBatchSize();
	if (batch_size > 0)
	{
		int actor_batch = GetActorBatchSize();
		int actor_input_size = GetActorInputSize();
		int actor_output_size = GetActorOutputSize();
		mBatchActorXBuffer.resize(actor_batch, actor_input_size);
		mBatchActorYBuffer.resize(actor_batch, actor_output_size);
	}
}

void cSARSATrainer::UpdateCriticOffsetScale()
{
	Eigen::VectorXd offset = GetCriticInputOffset();
	Eigen::VectorXd scale = GetCriticInputScale();
	mDataRecordX.CalcOffsetScale(mInputOffsetScaleTypes, mParams.mInputScaleMax, offset, scale);

	const auto& actor_net = GetActor();
	int state_size = GetStateSize();
	int action_size = GetActionSize();

	offset.segment(state_size, action_size) = actor_net->GetOutputOffset().segment(0, action_size);
	scale.segment(state_size, action_size) = actor_net->GetOutputScale().segment(0, action_size);
	SetCriticInputOffsetScale(offset, scale);

	if (EnableAsyncMode())
	{
		UpdateParamServerCriticInputOffsetScale(offset, scale);
	}
}

bool cSARSATrainer::EnableTargetNet() const
{
	return cCaclaTrainer::EnableTargetNet();
}

void cSARSATrainer::BuildTupleX(const tExpTuple& tuple, Eigen::VectorXd& out_x)
{
	int input_size = GetCriticInputSize();
	out_x.resize(input_size);
	out_x.segment(0, tuple.mStateBeg.size()) = tuple.mStateBeg;
	out_x.segment(tuple.mStateBeg.size(), tuple.mAction.size()) = tuple.mAction;
}

void cSARSATrainer::BuildCriticXNext(const tExpTuple& tuple, Eigen::VectorXd& out_x)
{
	int state_size = GetStateSize();
	int input_size = GetCriticInputSize();
	Eigen::VectorXd actor_x_next;
	BuildActorTupleXNext(tuple, actor_x_next);

	const auto& actor = GetActorTarget();
	Eigen::VectorXd action;
	actor->Eval(actor_x_next, action);

	out_x.resize(input_size);
	out_x.segment(0, state_size) = tuple.mStateEnd;
	out_x.segment(actor_x_next.size(), tuple.mAction.size()) = action;
}

void cSARSATrainer::CalcNewCumulativeRewardBatch(int net_id, const std::vector<int>& tuple_ids,
												Eigen::VectorXd& out_vals)
{
	const int num_data = static_cast<int>(tuple_ids.size());
	assert(num_data <= GetBatchSize());
	const auto& tar_net = GetTargetNet(net_id);

	int state_size = GetStateSize();
	int action_size = GetActionSize();
	Eigen::MatrixXd& X_next = mBatchXBuffer;

	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);

		Eigen::VectorXd x_next;
		BuildActorTupleXNext(tuple, x_next);
		mBatchActorXBuffer.row(i) = x_next;

		X_next.row(i).segment(0, state_size) = tuple.mStateEnd;
	}

	const auto& actor = GetActorTarget();
	actor->EvalBatch(mBatchActorXBuffer, mBatchActorYBuffer);
	X_next.block(0, state_size, num_data, action_size) = mBatchActorYBuffer.block(0,0, num_data, action_size);

	tar_net->EvalBatch(X_next, mBatchYBuffer);

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
			double v_end = mBatchYBuffer(i, 0);
#if defined(CLAMP_VAL)
			v_end = cMathUtil::Clamp(v_end, cQNetTrainer::gValClampMin, cQNetTrainer::gValClampMax);
#endif
			val = r + discount * v_end;
		}

		out_vals(i) = val;
	}
}

bool cSARSATrainer::CheckUpdateTarget(int iter) const
{
#if defined(ENABLE_LERP_TARGET)
	return true;
#else
	return cCaclaTrainer::CheckUpdateTarget(iter);
#endif
}

void cSARSATrainer::UpdateTargetNet()
{
	if (EnableTargetNet())
	{
		const auto& actor_net = GetActor();
		const auto& actor_tar = GetActorTarget();

#if defined(ENABLE_LERP_TARGET)
		actor_tar->BlendModel(*actor_net.get(), 1 - mTargetLerp, mTargetLerp);
#else
		actor_tar->CopyModel(*actor_net.get());
#endif

		for (int i = 0; i < GetNetPoolSize(); ++i)
		{
			const auto& critic_net = mNetPool[i];
			const auto& critic_tar = GetTargetNet(i);

#if defined(ENABLE_LERP_TARGET)
			critic_tar->BlendModel(*critic_net.get(), 1 - mTargetLerp, mTargetLerp);
#else
			critic_tar->CopyModel(*critic_net.get());
#endif	
		}
	}
}