/*
 * SimAdapter.h
 *
 *  Created on: 2017-01-20
 *      Author: gberseth
 */

#ifndef SIMADAPTER_H_
#define SIMADAPTER_H_
#include <vector>
#include <string>
#include "scenarios/Scenario.h"
#include "scenarios/ScenarioSimChar.h"

// #ifndef NDEBUG
#   define ASSERT(condition, message) \
    do { \
        if (! (condition)) { \
            std::cerr << "Assertion `" #condition "` failed in " << __FILE__ \
                      << " line " << __LINE__ << ": " << message << std::endl; \
            std::terminate(); \
        } \
    } while (false)
// #else
// #   define ASSERT(condition, message) do { } while (false)
// #endif


class cSimAdapter {
public:
	cSimAdapter(std::vector<std::string> args);
	virtual ~cSimAdapter();

	virtual void init();
	virtual void initEpoch();
	virtual double updateAction(std::vector<double> action);
	virtual double updateLLCAction(std::vector<double> action);
	/// Perform on simulation update
	virtual void update();
	virtual void finish();
	/// Returns the reward for executing action act
	virtual void act(std::vector<double> act);

	virtual double calcReward();
	virtual std::vector<double> calcRewards();
	virtual double calcVelocity();
	virtual bool hasStumbled();
	virtual double jointTorque();

	virtual bool endOfEpoch();
	virtual bool agentHasFallen();
	/// check whether or not the last action has completed and a new action is needed
	virtual bool needUpdatedAction();

	/// Stuff for interacting with simulation
	virtual void setRender(bool shouldRender);
	virtual void display();
	/// Interactive functions to doing things in the simulation, like reseting and throwing objects at the character
	virtual void onKeyEvent(int key, int mouseX, int mouseY);
	/// specify the relative path to TerrainRL
	virtual void setRelativeFilePath(std::string relativePath);
	/// specify the relative path to TerrainRL
	virtual void setRandomSeed(int seed);

	virtual std::vector<double> getState() const;
	virtual std::vector<double> getLLCState();


	virtual size_t getActionSpaceSize() const;
	virtual size_t getObservationSpaceSize() const;

	void handleUpdatedAction();

protected:
	size_t _lastControllerState;
	bool _render;
	std::shared_ptr<cScenario> _gScenario;
	std::shared_ptr<cScenarioSimChar> _scene;
	std::vector<std::string> _args;
	std::string _relativePath;
};

#endif /* SIMADAPTER_H_ */
