#pragma once

#include "learning/CaclaTrainer.h"

class cSARSATrainer : public cCaclaTrainer
{
public:
	
	cSARSATrainer();
	virtual ~cSARSATrainer();

	virtual void Clear();
	virtual void SetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetActorInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetActorOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetTargetLerp(double lerp);

	virtual void LoadActorModel(const std::string& model_file);
	virtual void LoadActorScale(const std::string& scale_file);

protected:
	double mTargetLerp;

	Eigen::MatrixXd mBatchActorXBuffer;
	Eigen::MatrixXd mBatchActorYBuffer;

	std::unique_ptr<cNeuralNet> mActorTargetNet;
	Eigen::VectorXd mActionMin;
	Eigen::VectorXd mActionMax;

	virtual void BuildActor(const std::string& net_file, const std::string& solver_file);
	virtual const std::unique_ptr<cNeuralNet>& GetActorTarget() const;

	virtual void InitBatchBuffers();
	virtual void UpdateCriticOffsetScale();

	virtual bool EnableTargetNet() const;

	virtual void BuildTupleX(const tExpTuple& tuple, Eigen::VectorXd& out_x);
	virtual void BuildCriticXNext(const tExpTuple& tuple, Eigen::VectorXd& out_x);

	virtual void CalcNewCumulativeRewardBatch(int net_id, const std::vector<int>& tuple_ids,
												Eigen::VectorXd& out_vals);

	virtual bool CheckUpdateTarget(int iter) const;
	virtual void UpdateTargetNet();
};