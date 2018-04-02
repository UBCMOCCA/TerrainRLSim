#pragma once

#include "learning/CaclaTrainer.h"

class cStochPGTrainer : public cCaclaTrainer
{
public:
	
	struct tNoiseParams
	{
		int mInputOffset;
		int mInputSize;
		double mStdev;
		Eigen::MatrixXd mKernel;

		int mNumEntropySamples;
		double mEntropyWeight;

		tNoiseParams();
	};

	cStochPGTrainer();
	virtual ~cStochPGTrainer();

	virtual void Init(const tParams& params);

	virtual void SetNoiseParams(const tNoiseParams& params);
	virtual const tNoiseParams& GetNoiseParams() const;

protected:
	
	tNoiseParams mNoiseParams;
	std::vector<std::unique_ptr<cNeuralNet>> mWorkerNets;

	virtual void BuildWorkerNets(int num_workers);

	virtual bool CheckActorTD(double td) const;
	virtual void BuildActorProblemY(const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);
	virtual void ApplyEntropy(const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);
	virtual void ApplyEntropyWorker(int row_idx_min, int row_idx_max, const Eigen::MatrixXd* X, 
									cNeuralNet* net, cNeuralNet::tProblem* out_prob);
	virtual void ApplyEntropyHelper(int row_idx, const Eigen::MatrixXd* X, 
									cNeuralNet* net, cNeuralNet::tProblem* out_prob);

	virtual void EvalKernelGradGaussian(const Eigen::VectorXd& x, cNeuralNet* net, Eigen::VectorXd& out_delta_a);
	virtual void EvalKernelGradInvQuad(const Eigen::VectorXd& x, cNeuralNet* net, Eigen::VectorXd& out_delta_a);

	virtual int GetNoiseInputOffset() const;
	virtual int GetNoiseInputSize() const;
};