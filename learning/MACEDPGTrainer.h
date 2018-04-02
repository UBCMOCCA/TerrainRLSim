#pragma once

#include "learning/DPGTrainer.h"
#include "MACEDPGLearner.h"

class cMACEDPGTrainer : public cDPGTrainer
{
public:
	static void CalcCriticVals(const cNeuralNet* critic_net, const Eigen::VectorXd& state, const Eigen::VectorXd& actions,
								Eigen::VectorXd& out_vals);

	static int GetMaxFragValIdx(const Eigen::VectorXd& vals);
	static double GetMaxFragVal(const Eigen::VectorXd& vals);
	static void GetFrag(const Eigen::VectorXd& actions, int frag_size, int a_idx, Eigen::VectorXd& out_action);
	static void SetFrag(const Eigen::VectorXd& action, int a_idx, int frag_size, Eigen::VectorXd& out_actions);
	static double GetVal(const Eigen::VectorXd& vals, int a_idx);
	static void SetVal(double val, int a_idx, Eigen::VectorXd& out_vals);
	static int CalcNumFrags(int param_size, int frag_size);

	cMACEDPGTrainer();
	virtual ~cMACEDPGTrainer();

	virtual void Clear();
	virtual void SetNumActionFrags(int num);
	virtual void SetActionFragSize(int size);
	virtual void SetTemp(double temp);

	virtual void RequestLearner(std::shared_ptr<cNeuralNetLearner>& out_learner);

protected:
	int mNumActionFrags;
	int mActionFragSize;
	double mTemp;
	Eigen::MatrixXd mBatchCriticValBuffer;
	Eigen::MatrixXd mBatchDPGBuffer;

	virtual void InitBatchBuffers();
	virtual int GetActionSize() const;
	virtual void CalcNewCumulativeRewardBatch(int net_id, const std::vector<int>& tuple_ids,
												Eigen::VectorXd& out_vals);
	virtual void BuildActorProblemY(const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);

	virtual void ProcessPoliGrad(const Eigen::VectorXd& action, int actor_id, Eigen::VectorXd& out_dpg) const;
	virtual void GetRegAction(int actor_id, Eigen::VectorXd& out_action) const;
	virtual void GetActionScale(int actor_id, Eigen::VectorXd& out_scale) const;
};