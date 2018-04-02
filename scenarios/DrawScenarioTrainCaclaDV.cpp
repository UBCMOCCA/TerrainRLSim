/*
 * DrawScenarioTrainCaclaDV.cpp
 *
 *  Created on: Oct 26, 2016
 *      Author: Glen
 */

#include "DrawScenarioTrainCaclaDV.h"
#include "scenarios/ScenarioTrainCaclaDV.h"

cDrawScenarioTrainCaclaDV::cDrawScenarioTrainCaclaDV(cCamera& cam) 
	: cDrawScenarioTrainCacla(cam)
{
	// TODO Auto-generated constructor stub

}

cDrawScenarioTrainCaclaDV::~cDrawScenarioTrainCaclaDV() {
	// TODO Auto-generated destructor stub
}

void cDrawScenarioTrainCaclaDV::BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioTrainCaclaDV>(new cScenarioTrainCaclaDV());
}

