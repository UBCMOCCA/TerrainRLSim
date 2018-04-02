/*
 * ScenarioTrainCaclaDV.h
 *
 *  Created on: Oct 26, 2016
 *      Author: Glen
 */

#ifndef SCENARIOS_SCENARIOTRAINCACLADV_H_
#define SCENARIOS_SCENARIOTRAINCACLADV_H_

#include "ScenarioTrainCacla.h"

class cScenarioTrainCaclaDV : public cScenarioTrainCacla {
public:
	cScenarioTrainCaclaDV();
	virtual ~cScenarioTrainCaclaDV();

	virtual std::string GetName() const;
	virtual void SetupLearner(const std::shared_ptr<cSimCharacter>& character, std::shared_ptr<cNeuralNetLearner>& out_learner) const;
	virtual void SetupTrainerParams(cNeuralNetTrainer::tParams& out_params) const;

protected:

	virtual void BuildTrainer(std::shared_ptr<cTrainerInterface>& out_trainer);
	virtual void SetupTrainerOffsetScale(const std::shared_ptr<cTrainerInterface>& out_trainer);
	virtual void SetupTrainerForwardDynamicsOffsetScale(const std::shared_ptr<cTrainerInterface>& out_trainer);
};

#endif /* SCENARIOS_SCENARIOTRAINCACLADV_H_ */
