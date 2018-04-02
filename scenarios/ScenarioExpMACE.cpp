#include "ScenarioExpMACE.h"

cScenarioExpMACE::cScenarioExpMACE()
{
}

cScenarioExpMACE::~cScenarioExpMACE()
{
}

std::string cScenarioExpMACE::GetName() const
{
	return "Exploration MACE";
}

void cScenarioExpMACE::RecordFlagsBeg(tExpTuple& out_tuple) const
{
	cScenarioExp::RecordFlagsBeg(out_tuple);

	bool exp_critic = CheckExpCritic();
	bool exp_actor = CheckExpActor();
	out_tuple.SetFlag(exp_critic, tExpTuple::eFlagExpCritic);
	out_tuple.SetFlag(exp_actor, tExpTuple::eFlagExpActor);
}

bool cScenarioExpMACE::CheckExpCritic() const
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

bool cScenarioExpMACE::CheckExpActor() const
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