/*
 * ACDLearner.cpp
 *
 *  Created on: Nov 18, 2016
 *      Author: Glen
 */

#include "ACDLearner.h"
#include "CaclaFDTrainer.h"

cACDLearner::cACDLearner(const std::shared_ptr<cNeuralNetTrainer>& trainer) : cACLearner(trainer)
{
	// TODO Auto-generated constructor stub
	mForwardDynamicsNet = nullptr;
}

cACDLearner::~cACDLearner() {
	// TODO Auto-generated destructor stub
}

void cACDLearner::OutputForwardDynamics(const std::string& filename) const
{
	mForwardDynamicsNet->OutputModel(filename);
	printf("Forward Dynamics model saved to %s\n", filename.c_str());
}

void cACDLearner::SyncNet()
{
	auto trainer = std::static_pointer_cast<cCaclaFDTrainer>(mTrainer);

	auto& actor_net = trainer->GetActor();
	mNet->CopyModel(*actor_net);

	if (HasCriticNet())
	{
		auto& critic_net = trainer->GetCritic();
		mCriticNet->CopyModel(*critic_net);
	}
	if (HasForwardDynamicsNet())
	{
		auto& forward_dynamics_net = trainer->GetForwardDynamics();
		mForwardDynamicsNet->CopyModel(*forward_dynamics_net);
	}
}

bool cACDLearner::HasForwardDynamicsNet() const
{
	return mForwardDynamicsNet != nullptr;
}

void cACDLearner::LoadForwardDynamicsNet(const std::string& net_file)
{
	if (HasForwardDynamicsNet())
	{
		mForwardDynamicsNet->LoadNet(net_file);
	}
}

void cACDLearner::LoadForwardDynamicsSolver(const std::string& solver_file)
{
	if (HasForwardDynamicsNet())
	{
		mForwardDynamicsNet->LoadSolver(solver_file);
	}
}

void cACDLearner::SetForwardDynamicsNet(cNeuralNet* net)
{
	assert(net != nullptr);
	mForwardDynamicsNet = net;
}

const cNeuralNet* cACDLearner::GetForwardDynamicsNet() const
{
	return mForwardDynamicsNet;
}

void cACDLearner::Init()
{
	auto trainer = std::static_pointer_cast<cCaclaFDTrainer>(mTrainer);
	LoadActorNet(trainer->GetActorNetFile());
	LoadCriticNet(trainer->GetCriticNetFile());
	LoadForwardDynamicsNet(trainer->GetForwardDynamicsNetFile());
	SyncNet();
}

