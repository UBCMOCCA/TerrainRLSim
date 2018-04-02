#pragma once

#include "OptScenarioPoliEval.h"

class cOptScenarioMimicEval : public cOptScenarioPoliEval
{
public:
	cOptScenarioMimicEval();
	virtual ~cOptScenarioMimicEval();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);

protected:

	std::string mOutputFile;
	std::string mOutputTauErrFile;

	virtual void BuildScene(int id, std::shared_ptr<cScenarioPoliEval>& out_scene) const;
	virtual void OutputResults() const;
	virtual void OutputTauErr(const std::string& out_file) const;
};