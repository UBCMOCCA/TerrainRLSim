#pragma once

#include "CaclaTrainer.h"
#include "DMACELearner.h"

class cDMACETrainer : public cCaclaTrainer
{
public:

	cDMACETrainer();
	virtual ~cDMACETrainer();

	virtual void Init(const tParams& params);

	virtual void SetNumActionFrags(int num);
	virtual void SetActionFragSize(int size);
	virtual int GetNumActionFrags() const;
	virtual int GetActionFragSize() const;

	virtual int GetActionSize() const;

	virtual void SetValBound(double bound);
	virtual void SetTemp(double temp);
	virtual void SetGateScale(double scale);
	virtual void RequestLearner(std::shared_ptr<cNeuralNetLearner>& out_learner);

protected:
	int mNumActionFrags;
	int mActionFragSize;
	double mTemp;
	double mGateScale;
	double mValBound;

	virtual void BuildActorProblemY(const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);
	virtual bool IsOffPolicy(int t) const;

	virtual int GetMaxFragIdxAux(const Eigen::VectorXd& params);
	virtual double GetMaxFragValAux(const Eigen::VectorXd& params);
	virtual void GetFragAux(const Eigen::VectorXd& params, int a_idx, Eigen::VectorXd& out_action);
	virtual void SetFragAux(const Eigen::VectorXd& frag, int a_idx, Eigen::VectorXd& out_params);
	virtual double GetValAux(const Eigen::VectorXd& params, int a_idx);
	virtual void SetValAux(double val, int a_idx, Eigen::VectorXd& out_params);

	virtual bool CheckActorTD(double td) const;
	virtual void ProcessPoliGradGate(const Eigen::VectorXd& val, Eigen::VectorXd& out_diff) const;
};