#pragma once

#include "scenarios/OptScenarioTrackMotion.h"

class cOptScenarioVelCtrl : public  cOptScenarioTrackMotion
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cOptScenarioVelCtrl(cOptimizer& optimizer);
	virtual ~cOptScenarioVelCtrl();

	virtual std::string GetName() const;
	

protected:
	virtual void BuildScene(int id, std::shared_ptr<cScenarioSimChar>& out_scene);
	virtual double CalcCurrErr(const std::shared_ptr<cScenarioSimChar>& scene);
	virtual double CalcCurrErrDog(const std::shared_ptr<cScenarioSimChar>& scene);
	virtual double CalcCurrErrRaptor(const std::shared_ptr<cScenarioSimChar>& scene);
	virtual double CalcCurrErrBiped(const std::shared_ptr<cScenarioSimChar>& scene);
	virtual double CalcCurrErrBiped3D(const std::shared_ptr<cScenarioSimChar>& scene);
	virtual double CalcCurrErrMonopedHopper(const std::shared_ptr<cScenarioSimChar>& scene);

	virtual double EvalPt(const tPoint& pt, const std::shared_ptr<cScenarioSimChar>& scene);
};