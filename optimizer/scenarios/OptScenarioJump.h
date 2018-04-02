#pragma once

#include "scenarios/OptScenarioTrackMotion.h"

class cOptScenarioJump : public  cOptScenarioTrackMotion
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cOptScenarioJump(cOptimizer& optimizer);
	virtual ~cOptScenarioJump();

	virtual std::string GetName() const;

protected:
	virtual void BuildScene(int id, std::shared_ptr<cScenarioSimChar>& out_scene);
	virtual void UpdateScene(double time_step, const std::shared_ptr<cScenarioSimChar>& scene);
	virtual void ResetJumpCompleted(const std::shared_ptr<cScenarioSimChar>& scene);

	virtual double CalcCurrErr(const std::shared_ptr<cScenarioSimChar>& scene);
	virtual double CalcCurrErrDog(const std::shared_ptr<cScenarioSimChar>& scene);
	virtual double CalcCurrErrRaptor(const std::shared_ptr<cScenarioSimChar>& scene);
	virtual double CalcCurrErrHopper(const std::shared_ptr<cScenarioSimChar>& scene);
	virtual double CalcCurrErrBiped(const std::shared_ptr<cScenarioSimChar>& scene);

	virtual double CalcJumpReward(const std::shared_ptr<cScenarioSimChar>& scene) const;

	virtual bool CheckTermination(double time, const std::shared_ptr<cScenarioSimChar>& scene) const;
	virtual double EvalPt(const tPoint& pt, const std::shared_ptr<cScenarioSimChar>& scene);
};
