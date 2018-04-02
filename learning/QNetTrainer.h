#pragma once

#include "learning/NeuralNetTrainer.h"

class cQNetTrainer : public cNeuralNetTrainer
{
public:
	
	static const double gValClampMin;
	static const double gValClampMax;

	cQNetTrainer();
	virtual ~cQNetTrainer();

	virtual void Init(const tParams& params);
	virtual void Clear();

protected:
	
	Eigen::MatrixXd mBatchXBuffer;
	Eigen::MatrixXd mBatchYBuffer0;
	Eigen::MatrixXd mBatchYBuffer1;

	virtual void InitBatchBuffers();
	virtual void InitProblem(cNeuralNet::tProblem& out_prob) const;
	virtual void SetupExpBufferParams(int buffer_size, cExpBuffer::tParams& out_params) const;

	virtual bool Step();
	virtual void BuildProblemY(int net_id, const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);
	virtual void BuildTupleY(int net_id, const tExpTuple& tuple, Eigen::VectorXd& out_y);

	virtual void UpdateCurrActiveNetID();
	virtual int GetNextActiveID() const;
	virtual int GetRandRefID(int id) const;
};
