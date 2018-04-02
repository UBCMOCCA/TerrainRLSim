/*
 * BipedControllerQ.h
 *
 *  Created on: 2016-06-28
 *      Author: gberseth
 */

#ifndef BIPEDCONTROLLERQ_H_
#define BIPEDCONTROLLERQ_H_

#include "sim/BipedController.h"
#include "sim/BaseControllerQ.h"

class cBipedControllerQ : public virtual cBipedController, public virtual cBaseControllerQ
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cBipedControllerQ();
	virtual ~cBipedControllerQ();
};

#endif /* BIPEDCONTROLLERQ_H_ */
