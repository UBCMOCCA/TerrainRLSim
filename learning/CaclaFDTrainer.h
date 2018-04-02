/*
 * CaclaFDTrainer.h
 *
 *  Created on: Nov 18, 2016
 *      Author: Glen
 */

#ifndef LEARNING_CACLAFDTRAINER_H_
#define LEARNING_CACLAFDTRAINER_H_

#include "learning/CaclaTrainer.h"

class cCaclaFDTrainer : public cCaclaTrainer {
public:
	cCaclaFDTrainer();
	virtual ~cCaclaFDTrainer();

	virtual const std::unique_ptr<cNeuralNet>& GetForwardDynamics() const;
	virtual const std::string& GetForwardDynamicsNetFile() const;
	virtual const std::string& GetForwardDynamicsSolverFile() const;
	virtual void Init(const tParams& params);
	virtual void InitForwardDynamicsProblem(cNeuralNet::tProblem& out_prob) const;
	virtual void OutputIntermediateModel(const std::string& filename) const;
	virtual void RequestLearner(std::shared_ptr<cNeuralNetLearner>& out_learner);

	virtual void SetInputOffsetScaleType(const std::vector<cNeuralNet::eOffsetScaleType>& scale_types);
	virtual void SetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);

	virtual void SetForwardDynamicsInputOffsetScaleType(const std::vector<cNeuralNet::eOffsetScaleType>& scale_types);
	virtual void SetForwardDynamicsInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetForwardDynamicsOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);


protected:
	size_t mForwardDynamicsIter;
	cNeuralNet::tProblem mForwardDynamicsProb;
	std::unique_ptr<cNeuralNet> mForwardDynamicsNet;
	std::vector<cNeuralNet::eOffsetScaleType> mForwardDynamicsInputOffsetScaleTypes;

	virtual void InitForwardDynamicsInputOffsetScaleTypes();
	virtual int GetServerForwardDynamicsID() const;
	virtual void UpdateForwardDynamics();
	virtual std::string GetForwardDynamicsFilename(const std::string& actor_filename) const;
	virtual void OutputModel(const std::string& filename) const;
	virtual void OutputForwardDynamics(const std::string& filename) const;
	virtual void BuildForwardDynamics(const std::string& net_file, const std::string& solver_file);
	virtual int GetForwardDynamicsOutputSize() const;
	virtual int GetForwardDynamicsInputSize() const;
	virtual void InitInputOffsetScaleTypes();
	virtual void UpdateOffsetScale();
	virtual void UpdateForwardDynamicsOffsetScale();
	virtual void BuildNets();
	virtual void LoadModels();
	virtual void LoadForwardDynamicsModel(const std::string& model_file);
	virtual bool Step();
	virtual void StepForwardDynamics();
	virtual void UpdateForwardDynamicsNet(const cNeuralNet::tProblem& prob);
	virtual bool BuildForwardDynamicsProblem(cNeuralNet::tProblem& out_prob);

	virtual bool HasForwardDynamicsInitModel() const;
	virtual void EvalForwardDynamics(const tExpTuple& tuple, Eigen::VectorXd& out_y);
	virtual void SyncNets();
	virtual void SyncForwardDynamicsNet();
	virtual void ResetSolvers();
	virtual void ResetForwardDynamicsSolver();
	virtual void UpdateParamServerForwardDynamicsInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void IncForwardDynamicsIter();

	virtual void BuildProblemStateAndAction(int net_id, const std::vector<int>& tuple_ids, cNeuralNet::tProblem& out_prob);
	virtual void BuildProblemNextState(int net_id, const std::vector<int>& tuple_ids,
		const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);

};

#endif /* LEARNING_CACLAFDTRAINER_H_ */
