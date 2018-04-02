#pragma once

#include "scenarios/ScenarioTrainDPG.h"

class cScenarioTrainMACEDPG : public cScenarioTrainDPG
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioTrainMACEDPG();
	virtual ~cScenarioTrainMACEDPG();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);

	virtual std::string GetName() const;

protected:
	double mTrainerTempScale;

	virtual void BuildTrainer(std::shared_ptr<cTrainerInterface>& out_trainer);
	virtual void BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const;
	virtual void GetFragParams(int& out_num_frags, int& out_frag_size) const;

	virtual void SetupLearner(const std::shared_ptr<cSimCharacter>& character, std::shared_ptr<cNeuralNetLearner>& out_learner) const;
	virtual void UpdateTrainer(const std::vector<tExpTuple>& tuples, int exp_id);
};