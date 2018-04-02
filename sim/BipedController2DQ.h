/*
 * BipedController2DQ.h
 *
 *  Created on: Sep 1, 2016
 *      Author: Glen
 */

#ifndef SIM_BIPEDCONTROLLER2DQ_H_
#define SIM_BIPEDCONTROLLER2DQ_H_

#include "sim/BipedController2D.h"
#include "sim/BaseControllerQ.h"

class cBipedController2DQ : public virtual cBipedController2D, public virtual cBaseControllerQ
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cBipedController2DQ();
	virtual ~cBipedController2DQ();
};

#endif /* SIM_BIPEDCONTROLLER2DQ_H_ */
