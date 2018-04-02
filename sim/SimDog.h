#pragma once

#include "sim/SimCharSoftFall.h"


class cSimDog : public cSimCharSoftFall
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum eJoint
	{
		eJointRoot,
		eJointSpine0,
		eJointSpine1,
		eJointSpine2,
		eJointSpine3,
		eJointTorso,
		eJointNeck0,
		eJointNeck1,
		eJointHead,
		eJointTail0,
		eJointTail1,
		eJointTail2,
		eJointTail3,
		eJointShoulder,
		eJointElbow,
		eJointWrist,
		eJointFinger,
		eJointHip,
		eJointKnee,
		eJointAnkle,
		eJointToe,
		eJointMax,
		eJointInvalid
	};

	cSimDog();
	virtual ~cSimDog();

	virtual bool HasStumbled() const;

protected:
	
	virtual bool FailFallMisc() const;
};