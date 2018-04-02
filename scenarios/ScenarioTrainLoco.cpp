#include "ScenarioTrainLoco.h"
#include "ScenarioExpLoco.h"

cScenarioTrainLoco::cScenarioTrainLoco()
{
}

cScenarioTrainLoco::~cScenarioTrainLoco()
{
}

void cScenarioTrainLoco::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioTrainCacla::ParseArgs(parser);
}

std::string cScenarioTrainLoco::GetName() const
{
	return "Train Locomotion";
}

void cScenarioTrainLoco::BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const
{
	out_exp = std::shared_ptr<cScenarioExp>(new cScenarioExpLoco());
}