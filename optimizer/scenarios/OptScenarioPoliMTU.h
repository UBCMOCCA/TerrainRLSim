#pragma once

#include <string>
#include "scenarios/ScenarioImitate.h"
#include "scenarios/OptScenarioMTU.h"

class cOptScenarioPoliMTU : public cScenario
{
public:
	cOptScenarioPoliMTU(cOptimizer& optimizer);
	virtual ~cOptScenarioPoliMTU();

	virtual void Init();
	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Reset();
	virtual void Clear();

	virtual void Run();
	virtual void SetTimeStep(double time_step);

	virtual std::string GetName() const;

protected:
	
	std::shared_ptr<cArgParser> mArgParser;
	std::string mPolicyOutputFile;
	std::string mMTUOutputFile;
	std::string mIntPolicyOutputFile;
	std::string mIntMTUOutputFile;

	int mNumTrainThreads;
	int mNumOptThreads;

	int mMaxIters;
	int mPoolSize;
	double mTimeStep;
	int mIter;

	cOptimizer& mOptimizer;
	std::shared_ptr<cScenarioImitate> mTrainScene;
	std::shared_ptr<cOptScenarioMTU> mOptScene;

	virtual void BuildTrainScene(std::shared_ptr<cScenarioImitate>& out_scene) const;
	virtual void BuildOptScene(std::shared_ptr<cOptScenarioMTU>& out_scene) const;
	virtual void SetupScenes();
	virtual void ResetParams();
	virtual bool HasInitPolicy() const;

	virtual void TrainPolicy();
	virtual void OptmizeMTU();

	virtual void OutputIntPoliModel(int iter, const std::string& output_path) const;
	virtual void OutputIntMTU(int iter, const std::string& output_path) const;
	virtual void OutputPoliModel(const std::string& output_path) const;
	virtual void OutputMTU(const std::string& output_path) const;
};