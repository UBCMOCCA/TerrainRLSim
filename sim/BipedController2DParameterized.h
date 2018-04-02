/*
 * BipedController2DParameterized.h
 *
 *  Created on: Oct 16, 2016
 *      Author: Glen
 */

#ifndef SIM_BIPEDCONTROLLER2DPARAMETERIZED_H_
#define SIM_BIPEDCONTROLLER2DPARAMETERIZED_H_

#include "BipedController2DCACLA.h"
#include "util/Rand.h"

class cBipedController2DParameterized : public cBipedController2DCACLA {
public:
	cBipedController2DParameterized();
	virtual ~cBipedController2DParameterized();

	virtual double CalcReward() const;
	virtual tVector GetTargetVel() const;
	int GetPoliStateSize() const;
	virtual void BuildPoliState(Eigen::VectorXd& out_state) const;
	virtual void Update(double time_step);

	virtual void getInternalState(Eigen::VectorXd& state) const;
	virtual void updateInternalState(Eigen::VectorXd& state);

	virtual std::string BuildTextInfoStr() const;

	virtual void Reset();

public:
	mutable double _target_vel;
	// cRand _rand;
};

#endif /* SIM_BIPEDCONTROLLER2DPARAMETERIZED_H_ */
