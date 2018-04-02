#include "CtStochController.h"

cCtStochController::cCtStochController() : cCtController()
{
}

cCtStochController::~cCtStochController()
{
}

int cCtStochController::GetPoliStateSize() const
{
	int state_size = cCtController::GetPoliStateSize() + GetNumNoiseUnits();
	return state_size;
}

int cCtStochController::GetPoliActionSize() const
{
	int size = cCtController::GetPoliActionSize();
	size += GetNumNoiseUnits();
	return size;
}

void cCtStochController::BuildNNInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const
{
	cCtController::BuildNNInputOffsetScaleTypes(out_types);

	int offset = GetNoiseStateOffset();
	int size = GetNoiseStateSize();
	for (int i = 0; i < size; ++i)
	{
		out_types[offset + i] = cNeuralNet::eOffsetScaleTypeFixed;
	}
}

void cCtStochController::BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	cCtController::BuildActorOutputOffsetScale(out_offset, out_scale);

	int offset = GetNoiseActionOffset();
	int size = GetNoiseActionSize();
	out_offset.segment(offset, size) = Eigen::VectorXd::Zero(size);
	out_scale.segment(offset, size) = Eigen::VectorXd::Ones(size);
}

void cCtStochController::GetPoliActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	cCtController::GetPoliActionBounds(out_min, out_max);

	Eigen::VectorXd noise_min;
	Eigen::VectorXd noise_max;
	BuildActionNoiseBounds(noise_min, noise_max);

	int noise_offset = GetNoiseActionOffset();
	int noise_size = GetNoiseActionSize();
	out_min.segment(noise_offset, noise_size) = noise_min;
	out_max.segment(noise_offset, noise_size) = noise_max;
}

void cCtStochController::BuildPoliState(Eigen::VectorXd& out_state) const
{
	cCtController::BuildPoliState(out_state);

	int noise_offset = GetNoiseStateOffset();
	int noise_size = GetNoiseStateSize();
	out_state.segment(noise_offset, noise_size).setZero();
}

void cCtStochController::ExploreAction(Eigen::VectorXd& state, tAction& out_action)
{
	ApplyExpNoiseInternal(state);
	cCtController::ExploreAction(state, out_action);
}

void cCtStochController::ApplyExpNoiseInternal(Eigen::VectorXd& out_state) const
{
	const double noise_bound = 3 * mExpParams.mNoiseInternal;
	//const double noise_bound = std::numeric_limits<double>::infinity();

	int noise_offset = GetNoiseStateOffset();
	int noise_size = GetNoiseStateSize();
	for (int i = 0; i < noise_size; ++i)
	{
		double curr_noise = 0;
		do
		{
			curr_noise = cMathUtil::RandDoubleNorm(0, mExpParams.mNoiseInternal);
		} while (std::abs(curr_noise) > noise_bound);

		out_state[noise_offset + i] = curr_noise;
	}
}

int cCtStochController::GetNumNoiseUnits() const
{
	return mNet->GetInputSize() - cCtController::GetPoliStateSize();
}

int cCtStochController::GetNoiseStateOffset() const
{
	return cCtController::GetPoliStateSize();
}

int cCtStochController::GetNoiseStateSize() const
{
	return GetNumNoiseUnits();
}

int cCtStochController::GetNoiseActionOffset() const
{
	return cCtController::GetPoliActionSize();
}

int cCtStochController::GetNoiseActionSize() const
{
	return GetNumNoiseUnits();
}

void cCtStochController::BuildActionNoiseBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	//const double noise_bound = 10;
	const double noise_bound = std::numeric_limits<double>::infinity();

	int noise_size = GetNoiseActionSize();
	out_min = -noise_bound * Eigen::VectorXd::Ones(noise_size);
	out_max = noise_bound * Eigen::VectorXd::Ones(noise_size);
}