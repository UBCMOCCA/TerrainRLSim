#pragma once

#include "scenarios/ScenarioTrainCacla.h"

class cScenarioTrainDMACE : public cScenarioTrainCacla
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioTrainDMACE();
	virtual ~cScenarioTrainDMACE();

	virtual std::string GetName() const;

protected:

	virtual void BuildTrainer(std::shared_ptr<cTrainerInterface>& out_trainer);
	virtual void BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const;

	virtual void SetupTrainerCriticOffsetScale(const std::shared_ptr<cTrainerInterface>& out_trainer);
	virtual void SetupTrainerActorOffsetScale(const std::shared_ptr<cTrainerInterface>& out_trainer);

	virtual void SetupLearner(const std::shared_ptr<cSimCharacter>& character, std::shared_ptr<cNeuralNetLearner>& out_learner) const;
	virtual void GetFragParams(int& out_num_frags, int& out_frag_size) const;

	virtual void UpdateTrainer(const std::vector<tExpTuple>& tuples, int exp_id);
};