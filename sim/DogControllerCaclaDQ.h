/*
 * DogControllerCaclaDQ.h
 *
 *  Created on: 2016-06-23
 *      Author: gberseth
 */

#ifndef DOGCONTROLLERCACLADQ_H_
#define DOGCONTROLLERCACLADQ_H_

#include "DogControllerCacla.h"
#include "util/Rand.h"

class cDogControllerCaclaDQ : public cDogControllerCacla {
public:
	cDogControllerCaclaDQ();
	virtual ~cDogControllerCaclaDQ();

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
	mutable double _torque_sum;
	mutable size_t _num_frames;
	// cRand _rand;
};

#endif /* DOGCONTROLLERCACLADQ_H_ */
