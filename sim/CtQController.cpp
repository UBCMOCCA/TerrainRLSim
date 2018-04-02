#include "CtQController.h"

const int gBoltzSamples = 16;

cCtQController::cCtQController() : cCtController()
{
}

cCtQController::~cCtQController()
{
}

void cCtQController::Init(cSimCharacter* character)
{
	cCtController::Init(character);
	InitBatchBuffers();
}

int cCtQController::GetCriticInputSize() const
{
	return GetPoliStateSize() + GetPoliActionSize();
}

void cCtQController::BuildCriticInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	Eigen::VectorXd state_offset;
	Eigen::VectorXd state_scale;
	Eigen::VectorXd action_offset;
	Eigen::VectorXd action_scale;
	cCtController::BuildCriticInputOffsetScale(state_offset, state_scale);
	BuildActorOutputOffsetScale(action_offset, action_scale);

	int state_size = GetPoliStateSize();
	int action_size = GetPoliActionSize();
	assert(state_offset.size() == state_size);
	assert(state_scale.size() == state_size);
	assert(action_offset.size() == action_size);
	assert(action_scale.size() == action_size);

	out_offset.resize(state_size + action_size);
	out_scale.resize(state_size + action_size);
	out_offset.segment(0, state_size) = state_offset;
	out_scale.segment(0, state_size) = state_scale;
	out_offset.segment(state_size, action_size) = action_offset;
	out_scale.segment(state_size, action_size) = action_scale;
}

void cCtQController::InitBatchBuffers()
{
	int critic_input_size = GetCriticInputSize();
	int critic_output_size = GetCriticOutputSize();
	mCriticInputBatchBuffer.resize(gBoltzSamples, critic_input_size);
	mCriticOutputBatchBuffer.resize(gBoltzSamples, critic_output_size);
	mBoltzVals.resize(gBoltzSamples);
}

void cCtQController::BuildCriticInput(Eigen::VectorXd& out_x) const
{
	int state_size = GetPoliStateSize();
	int action_size = GetPoliActionSize();
	out_x.resize(state_size + action_size);
	out_x.segment(0, state_size) = mPoliState;

	Eigen::VectorXd action;
	RecordPoliAction(action);
	out_x.segment(state_size, action_size) = action;
}

void cCtQController::ExploreAction(Eigen::VectorXd& state, tAction& out_action)
{
	if (ValidCritic())
	{
		ExploreActionBoltz(state, out_action);
	}
	else
	{
		ExploitPolicy(state, out_action);
		ApplyExpNoise(out_action);
	}
}

void cCtQController::ExploreActionBoltz(Eigen::VectorXd& state, tAction& out_action)
{
	ExploitPolicy(state, out_action);
	int state_size = GetPoliStateSize();
	int action_size = GetPoliActionSize();

	for (int i = 0; i < gBoltzSamples; ++i)
	{
		auto curr_row = mCriticInputBatchBuffer.row(i);
		curr_row.segment(0, state_size) = state;
	}

	for (int i = 0; i < gBoltzSamples; ++i)
	{
		tAction curr_action = out_action;
		ApplyExpNoise(curr_action);

		auto curr_row = mCriticInputBatchBuffer.row(i);
		curr_row.segment(state_size, action_size) = curr_action.mParams;
	}

	mCriticNet->EvalBatch(mCriticInputBatchBuffer, mCriticOutputBatchBuffer);

	cMathUtil::CalcSoftmax(mCriticOutputBatchBuffer, mExpParams.mTemp, mBoltzVals);
	int selected_a = cMathUtil::SampleDiscreteProb(mBoltzVals);

#if defined(ENABLE_DEBUG_PRINT)
	printf("Boltz Exp:\n");
	printf("Selected: %i\n", selected_a);
	printf("Min: %.5f\n", mCriticOutputBatchBuffer.minCoeff());
	printf("Max: %.5f\n", mCriticOutputBatchBuffer.maxCoeff());
	for (int i = 0; i < static_cast<int>(mCriticOutputBatchBuffer.size()); ++i)
	{
		printf("%i: %.5f (%.5f)\n", i, mCriticOutputBatchBuffer(i, 0), mBoltzVals[i]);
	}
	printf("\n");
#endif

	auto selected_row = mCriticInputBatchBuffer.row(selected_a);
	out_action.mParams = selected_row.segment(state_size, action_size);
}