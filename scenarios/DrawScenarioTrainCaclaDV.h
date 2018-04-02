/*
 * DrawScenarioTrainCaclaDV.h
 *
 *  Created on: Oct 26, 2016
 *      Author: Glen
 */

#ifndef SCENARIOS_DRAWSCENARIOTRAINCACLADV_H_
#define SCENARIOS_DRAWSCENARIOTRAINCACLADV_H_

#include "DrawScenarioTrainCacla.h"

class cDrawScenarioTrainCaclaDV : public cDrawScenarioTrainCacla {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioTrainCaclaDV(cCamera& cam);
	virtual ~cDrawScenarioTrainCaclaDV();

protected:

	void BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const;
};

#endif /* SCENARIOS_DRAWSCENARIOTRAINCACLADV_H_ */
