#include "StochPGTrainer.h"
#include <thread>

cStochPGTrainer::tNoiseParams::tNoiseParams()
{
	mInputOffset = 0;
	mInputSize = 0;
	mStdev = 1;
	mKernel.resize(0, 0);
}

cStochPGTrainer::cStochPGTrainer()
{
}

cStochPGTrainer::~cStochPGTrainer()
{
}

void cStochPGTrainer::Init(const tParams& params)
{
	cCaclaTrainer::Init(params);

	BuildWorkerNets(mParams.mNumThreads);
}

void cStochPGTrainer::SetNoiseParams(const tNoiseParams& params)
{
	mNoiseParams = params;
}

const cStochPGTrainer::tNoiseParams& cStochPGTrainer::GetNoiseParams() const
{
	return mNoiseParams;
}

void cStochPGTrainer::BuildWorkerNets(int num_workers)
{
	const std::string& actor_net_file = GetActorNetFile();
	mWorkerNets.resize(num_workers);

	for (int i = 0; i < num_workers; ++i)
	{
		auto& curr_worker = mWorkerNets[i];
		curr_worker = std::unique_ptr<cNeuralNet>(new cNeuralNet());
		curr_worker->LoadNet(actor_net_file);
	}
}

bool cStochPGTrainer::CheckActorTD(double td) const
{
	return cCaclaTrainer::CheckActorTD(td);
}

void cStochPGTrainer::BuildActorProblemY(const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob)
{
	cCaclaTrainer::BuildActorProblemY(tuple_ids, X, out_prob);
	ApplyEntropy(X, out_prob);
}

void cStochPGTrainer::ApplyEntropy(const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob)
{
	int num_tuples = static_cast<int>(X.rows());
	int num_threads = static_cast<int>(mWorkerNets.size());
	num_threads = std::min(num_threads, num_tuples);

	int tups_per_thread = num_tuples / num_threads;
	int tups_remain = num_tuples % num_threads;

	std::vector<std::thread> threads(num_threads);

	for (int i = 0; i < num_threads; ++i)
	{
		const auto& worker_net = mWorkerNets[i];
		int row_idx_beg = tups_per_thread * i;
		row_idx_beg += (i < tups_remain) ? i : tups_remain;
		int num_rows = (i < tups_remain) ? (tups_per_thread + 1) : tups_per_thread;
		int row_idx_end = row_idx_beg + num_rows;

		std::thread& curr_thread = threads[i];
		curr_thread = std::thread(&cStochPGTrainer::ApplyEntropyWorker, this, row_idx_beg, row_idx_end, &X, worker_net.get(), &out_prob);
	}

	for (int i = 0; i < num_threads; ++i)
	{
		threads[i].join();
	}
}

void cStochPGTrainer::ApplyEntropyWorker(int row_idx_min, int row_idx_max, const Eigen::MatrixXd* X, 
										cNeuralNet* net, cNeuralNet::tProblem* out_prob)
{
	const auto& actor = GetActor();
	net->CopyModel(*actor);

	for (int i = row_idx_min; i < row_idx_max; ++i)
	{
		ApplyEntropyHelper(i, X, net, out_prob);
	}
}

void cStochPGTrainer::ApplyEntropyHelper(int row_idx, const Eigen::MatrixXd* X, 
										cNeuralNet* net, cNeuralNet::tProblem* out_prob)
{
	Eigen::VectorXd x = X->row(row_idx);
	Eigen::VectorXd dA;
	//EvalKernelGradGaussian(x, net, dA);
	EvalKernelGradInvQuad(x, net, dA);

	int action_size = static_cast<int>(dA.size());
	dA = dA.cwiseQuotient(out_prob->mW.block(row_idx, 0, 1, action_size).transpose());
	dA = mParams.mEntropyWeight * dA;
	out_prob->mY.block(row_idx, 0, 1, action_size) += dA.transpose();
}

void cStochPGTrainer::EvalKernelGradGaussian(const Eigen::VectorXd& x, cNeuralNet* net, Eigen::VectorXd& out_delta_a)
{
	int num_samples = mParams.mNumEntropySamples;
	int noise_offset = GetNoiseInputOffset();
	int noise_size = GetNoiseInputSize();
	int y_size = GetActionSize();
	int action_size = y_size - noise_size;

	out_delta_a = Eigen::VectorXd::Zero(action_size);
	Eigen::VectorXd curr_x = x;
	auto x_noise = curr_x.segment(noise_offset, noise_size);

	Eigen::VectorXd y = Eigen::VectorXd::Zero(y_size);
	net->Eval(curr_x, y);
	Eigen::VectorXd curr_a = y.segment(0, action_size);

	Eigen::MatrixXd samples = Eigen::MatrixXd::Zero(num_samples, action_size);
	std::vector<double> sample_dists(num_samples);

	for (int s = 0; s < num_samples; ++s)
	{
		for (int j = 0; j < noise_size; ++j)
		{
			x_noise(j) = cMathUtil::RandDoubleNorm(0, mNoiseParams.mStdev);
		}

		net->Eval(curr_x, y);
		Eigen::VectorXd sample_a = y.segment(0, action_size);
		Eigen::VectorXd curr_delta_a = curr_a - sample_a;
		double d = curr_delta_a.dot(mNoiseParams.mKernel * curr_delta_a);
		sample_dists[s] = d;
		samples.row(s) = curr_delta_a;
	}

	std::vector<double> sample_dists_sorted = sample_dists;
	std::sort(sample_dists_sorted.begin(), sample_dists_sorted.end());
	double med = sample_dists_sorted[(sample_dists_sorted.size() - 1) / 2];
	double h = med;

	for (int s = 0; s < num_samples; ++s)
	{
		double d = sample_dists[s];
		double k = std::exp(-d / h);

		Eigen::VectorXd curr_delta_a = samples.row(s);
		//curr_delta_a = (2 * k / h) * (mNoiseParams.mKernel * curr_delta_a);
		curr_delta_a = (2 * k / h) * curr_delta_a; // hack
		out_delta_a += curr_delta_a;
	}

	out_delta_a /= num_samples;
}

void cStochPGTrainer::EvalKernelGradInvQuad(const Eigen::VectorXd& x, cNeuralNet* net, Eigen::VectorXd& out_delta_a)
{
	const double h = 1 / mParams.mEntropyKernelWidth;

	int num_samples = mParams.mNumEntropySamples;
	int noise_offset = GetNoiseInputOffset();
	int noise_size = GetNoiseInputSize();
	int y_size = GetActionSize();
	int action_size = y_size - noise_size;

	out_delta_a = Eigen::VectorXd::Zero(action_size);
	Eigen::VectorXd curr_x = x;
	auto x_noise = curr_x.segment(noise_offset, noise_size);

	Eigen::VectorXd y = Eigen::VectorXd::Zero(y_size);
	net->Eval(curr_x, y);
	Eigen::VectorXd curr_a = y.segment(0, action_size);

	for (int s = 0; s < num_samples; ++s)
	{
		for (int j = 0; j < noise_size; ++j)
		{
			x_noise(j) = cMathUtil::RandDoubleNorm(0, mNoiseParams.mStdev);
		}

		net->Eval(curr_x, y);
		Eigen::VectorXd sample_a = y.segment(0, action_size);
		Eigen::VectorXd curr_delta_a = curr_a - sample_a;

		double d = curr_delta_a.dot(mNoiseParams.mKernel * curr_delta_a);
		d = h * h * d;
		double k = 1 / (d + 1);
		k *= k;
		k *= 2 * h * h;
		curr_delta_a *= k;

		out_delta_a += curr_delta_a;
	}

	out_delta_a /= num_samples;
}

int cStochPGTrainer::GetNoiseInputOffset() const
{
	return mNoiseParams.mInputOffset;
}

int cStochPGTrainer::GetNoiseInputSize() const
{
	return mNoiseParams.mInputSize;
}