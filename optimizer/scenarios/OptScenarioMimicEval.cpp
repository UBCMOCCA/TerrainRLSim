#include "OptScenarioMimicEval.h"
#include "scenarios/ScenarioMimicEval.h"
#include "util/FileUtil.h"

cOptScenarioMimicEval::cOptScenarioMimicEval()
{
	mOutputTauErrFile = "";
}

cOptScenarioMimicEval::~cOptScenarioMimicEval()
{
}

void cOptScenarioMimicEval::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cOptScenarioPoliEval::ParseArgs(parser);

	parser->ParseString("tau_err_file", mOutputTauErrFile);
}

void cOptScenarioMimicEval::BuildScene(int id, std::shared_ptr<cScenarioPoliEval>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioMimicEval>(new cScenarioMimicEval());
}

void cOptScenarioMimicEval::OutputResults() const
{
	cOptScenarioPoliEval::OutputResults();

	if (mOutputTauErrFile != "")
	{
		OutputTauErr(mOutputTauErrFile);
	}
}

void cOptScenarioMimicEval::OutputTauErr(const std::string& out_file) const
{
	double tau_err = 0;
	int total_count = 0;
	for (int i = 0; i < GetPoolSize(); ++i)
	{
		double curr_err = 0;
		int curr_count = 0;
		auto mimic_scene = std::dynamic_pointer_cast<cScenarioMimicEval>(mEvalPool[i]);
		mimic_scene->GetTauErrResult(curr_err, curr_count);

		tau_err = cMathUtil::AddAverage(tau_err, total_count, curr_err, curr_count);
		total_count += curr_count;
	}

	if (total_count > 0)
	{
		std::string str = std::to_string(tau_err);
		str += "\n";
		bool succ = cFileUtil::AppendText(str, out_file);

		if (!succ)
		{
			printf("Failed to output results to %s\n", out_file.c_str());
		}
	}
	
}