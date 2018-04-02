#pragma once

#include "learning/SARSATrainer.h"

class cDPGTrainer : public cSARSATrainer
{
public:
	cDPGTrainer();
	virtual ~cDPGTrainer();

	virtual void SetQDiff(double q_diff);
	virtual void SetDPGReg(double reg);

	virtual void CalcDPG(const tExpTuple& tuple, Eigen::VectorXd& out_dpg);
	virtual double CheckDPG(const tExpTuple& tuple, double delta = 0.001);

protected:
	double mQDiff;
	double mDPGReg;

	virtual void FetchActorMinibatch(int batch_size, std::vector<int>& out_batch);
	
	virtual void BuildTupleActorY(const tExpTuple& tuple, Eigen::VectorXd& out_y);
	
	virtual double GetQDiff() const;
	virtual void UpdateActor();
	virtual void BuildActorProblem(cNeuralNet::tProblem& out_prob);
	virtual void BuildActorProblemY(const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);
	
	virtual void UpdateBuffers(int t);

	virtual void CalcDPGIntern(const tExpTuple& tuple, Eigen::VectorXd& out_dpg);
	virtual void CalcDPGNum(const tExpTuple& tuple, double delta, Eigen::VectorXd& out_dpg);
	virtual void ProcessPoliGrad(const Eigen::VectorXd& action, Eigen::VectorXd& out_dpg) const;
	virtual void GetRegAction(Eigen::VectorXd& out_action) const;
	virtual void GetActionScale(Eigen::VectorXd& out_scale) const;
};