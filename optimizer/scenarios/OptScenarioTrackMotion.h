#pragma once

#include "scenarios/OptScenarioSimChar.h"

class cOptScenarioTrackMotion : public  cOptScenarioSimChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cOptScenarioTrackMotion(cOptimizer& optimizer);
	virtual ~cOptScenarioTrackMotion();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);

	virtual int GetDimensions() const;
	virtual void InitPt(tPoint& pt) const;
	virtual void InitScale(tPoint& scale) const;

	virtual bool IsThreadSafe() const;
	virtual void OutputPt(FILE*, const tPoint& pt) const;
	
	virtual std::string GetName() const;

protected:
	double mMaxTime;
	int mEvalSamples;
	
	virtual void BuildScene(int id, std::shared_ptr<cScenarioSimChar>& out_scene);

	virtual double CalcCurrErr(const std::shared_ptr<cScenarioSimChar>& scene);
	virtual double CalcCurrErrDog(const std::shared_ptr<cScenarioSimChar>& scene);
	virtual double CalcCurrErrRaptor(const std::shared_ptr<cScenarioSimChar>& scene);
	virtual double CalcCurrErrHopper(const std::shared_ptr<cScenarioSimChar>& scene);

	virtual double GetFallPenalty() const;
	virtual double GetStumblePenalty() const;

	virtual const std::shared_ptr<cCharController>& GetDefaultController() const;
	
	virtual double EvalPt(const tPoint& pt, const std::shared_ptr<cScenarioSimChar>& scene);
	virtual bool CheckTermination(double time, const std::shared_ptr<cScenarioSimChar>& scene) const;

	virtual int GetTargetCtrlID() const;
	virtual const std::vector<int>& GetTargetActions() const;
};