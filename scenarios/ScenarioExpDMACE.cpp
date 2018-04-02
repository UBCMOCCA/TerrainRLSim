#include "ScenarioExpDMACE.h"

cScenarioExpDMACE::cScenarioExpDMACE()
{
}

cScenarioExpDMACE::~cScenarioExpDMACE()
{
}

std::string cScenarioExpDMACE::GetName() const
{
	return "Exploration DMACE";
}

void cScenarioExpDMACE::RecordFlagsBeg(tExpTuple& out_tuple) const
{
	cScenarioExpCacla::RecordFlagsBeg(out_tuple);

	bool exp_critic = CheckExpCritic();
	bool exp_actor = CheckExpActor();
	out_tuple.SetFlag(exp_critic, tExpTuple::eFlagExpCritic);
	out_tuple.SetFlag(exp_actor, tExpTuple::eFlagExpActor);
}

bool cScenarioExpDMACE::CheckExpCritic() const
{
	bool exp = false;
	auto ctrl = GetCharacter()->GetController();
	std::shared_ptr<cBaseControllerMACE const> ac_int = std::dynamic_pointer_cast<cBaseControllerMACE const>(ctrl);

	if (ac_int != nullptr)
	{
		exp = ac_int->IsExpCritic();
	}
	else
	{
		assert(false); // controller does not implement actor-critic interface
	}

	return exp;
}

bool cScenarioExpDMACE::CheckExpActor() const
{
	bool exp = false;
	auto ctrl = GetCharacter()->GetController();
	std::shared_ptr<cBaseControllerMACE const> ac_int = std::dynamic_pointer_cast<cBaseControllerMACE const>(ctrl);

	if (ac_int != nullptr)
	{
		exp = ac_int->IsExpActor();
	}
	else
	{
		assert(false); // controller does not implement actor-critic interface
	}

	return exp;
}