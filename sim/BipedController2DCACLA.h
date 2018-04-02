/*
 * BipedController2DCACLA.h
 *
 *  Created on: 2016-09-06
 *      Author: gberseth
 */

#ifndef BIPEDCONTROLLER2DCACLA_H_
#define BIPEDCONTROLLER2DCACLA_H_

#include "sim/BaseControllerCacla.h"
#include "sim/BipedController2D.h"

class cBipedController2DCACLA : public cBipedController2D, public virtual cBaseControllerCacla {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cBipedController2DCACLA();
	virtual ~cBipedController2DCACLA();

	virtual void Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file);
	virtual void Update(double time_step);
	virtual double CalcReward() const;

	mutable double _torque_sum;
	mutable size_t _num_frames;

protected:
	virtual void UpdateAction();
	virtual bool IsCurrActionCyclic() const;
};

#endif /* BIPEDCONTROLLER2DCACLA_H_ */
