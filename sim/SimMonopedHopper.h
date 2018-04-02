/*
 * SimMonopedHopper.h
 *
 *  Created on: 2015-12-01
 *      Author: gberseth
 */

#ifndef SIMMONOPEDHOPPER_H_
#define SIMMONOPEDHOPPER_H_

#include "sim/SimCharSoftFall.h"

class cSimMonopedHopper : public cSimCharSoftFall
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum eJoint
	{
		eJointRoot,
		eJointHip,
		eJointKnee,
		eJointMax,
		eJointInvalid
	};

	cSimMonopedHopper();
	virtual ~cSimMonopedHopper();

	virtual bool HasStumbled() const;

protected:
	
	virtual bool FailFallMisc() const;
};

#endif /* SIMMONOPEDHOPPER_H_ */
