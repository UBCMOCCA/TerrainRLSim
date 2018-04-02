/*
 * BipedController2DCaclaFD.h
 *
 *  Created on: Nov 18, 2016
 *      Author: Glen B
 */

#ifndef SIM_BIPEDCONTROLLER2DCACLAFD_H_
#define SIM_BIPEDCONTROLLER2DCACLAFD_H_

#include "sim/BaseControllerCaclaFD.h"
#include "sim/BipedController2D.h"

class cBipedController2DCaclaFD : public cBipedController2D, public virtual cBaseControllerCaclaFD {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cBipedController2DCaclaFD();
	virtual ~cBipedController2DCaclaFD();

	virtual void Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file);
	virtual void Update(double time_step);
	virtual double CalcReward() const;

	mutable double _torque_sum;
	mutable size_t _num_frames;

protected:
	virtual void UpdateAction();
	virtual bool IsCurrActionCyclic() const;
};

#endif /* SIM_BIPEDCONTROLLER2DCACLAFD_H_ */
