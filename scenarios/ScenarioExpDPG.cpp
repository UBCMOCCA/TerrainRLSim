#include "ScenarioExpDPG.h"

cScenarioExpDPG::cScenarioExpDPG()
{
}

cScenarioExpDPG::~cScenarioExpDPG()
{
}

std::string cScenarioExpDPG::GetName() const
{
	return "Exploration DPG";
}

bool cScenarioExpDPG::EnableRandInitAction() const
{
	return cScenarioExpCacla::EnableRandInitAction();
	//return true;
}