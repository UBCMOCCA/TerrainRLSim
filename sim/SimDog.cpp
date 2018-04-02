#include "sim/SimDog.h"
#include <iostream>

cSimDog::cSimDog()
{
}

cSimDog::~cSimDog()
{
}

bool cSimDog::HasStumbled() const
{
	bool stumbled = false;
	for (int i = 0; i < cSimDog::eJointMax; ++i)
	{
		cSimDog::eJoint joint_id = static_cast<cSimDog::eJoint>(i);

		if (joint_id != cSimDog::eJointToe 
			&& joint_id != cSimDog::eJointFinger
			&& joint_id != cSimDog::eJointAnkle
			&& joint_id != cSimDog::eJointWrist)
		{
			const auto& curr_part = GetBodyPart(joint_id);
			bool contact = curr_part->IsInContact();
			if (contact)
			{
				stumbled = true;
				break;
			}
		}
	}
	return stumbled;
}

bool cSimDog::FailFallMisc() const
{
	bool fallen = false;
	tQuaternion root_q = GetRootRotation();
	double root_theta = cMathUtil::QuatTheta(root_q);

	bool flipped = std::abs(root_theta) > M_PI * 0.8;
	fallen |= flipped;

	return fallen;
}