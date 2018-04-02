#pragma once

#include "scenarios/ScenarioTrain.h"

class cScenarioMimic : public cScenarioTrain
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioMimic();
	virtual ~cScenarioMimic();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void EnableTraining(bool enable);

	virtual std::string GetName() const;

protected:

	double mCoachActiveProb;
	double mInitCoachActiveProb;

	virtual void BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const;
	virtual void BuildTrainer(std::shared_ptr<cTrainerInterface>& out_trainer);
	virtual void ResetScenePool();

	virtual int GetCharCtrlID() const;
	virtual int GetCoachCtrlID() const;
	virtual void UpdateExpScene(double time_step, int exp_id, std::shared_ptr<cScenarioExp>& out_exp, bool& out_done);
	virtual void SetupLearner(const std::shared_ptr<cSimCharacter>& character, std::shared_ptr<cNeuralNetLearner>& out_learner) const;
	virtual void SetupTrainerOffsetScale(const std::shared_ptr<cTrainerInterface>& out_trainer);

	virtual void UpdateExpSceneRates(int exp_id, std::shared_ptr<cScenarioExp>& out_exp) const;
	virtual double CalcCoachBlend(int iters) const;
	virtual double CalcCoachActiveProb(int iters) const;

	virtual void PrintLearnerInfo(int exp_id) const;
};