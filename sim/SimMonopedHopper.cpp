/*
 * SimMonopedHopper.cpp
 *
 *  Created on: 2015-12-01
 *      Author: gberseth
 */

#include "sim/SimMonopedHopper.h"

cSimMonopedHopper::cSimMonopedHopper() {
	// TODO Auto-generated constructor stub

}

cSimMonopedHopper::~cSimMonopedHopper() {
	// TODO Auto-generated destructor stub
}

#include "sim/SimMonopedHopper.h"
#include <iostream>


bool cSimMonopedHopper::HasStumbled() const
{
	bool stumbled = false;
	for (int i = 0; i < cSimMonopedHopper::eJointMax; ++i)
	{
		cSimMonopedHopper::eJoint joint_id = static_cast<cSimMonopedHopper::eJoint>(i);

		if (joint_id != cSimMonopedHopper::eJointKnee)
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

bool cSimMonopedHopper::FailFallMisc() const
{
	bool fallen = false;

	const tVector ref_axis = tVector(0, 0, 1, 0);
	tVector root_axis;
	double root_theta;
	GetRootRotation(root_axis, root_theta);

	if (root_axis.dot(ref_axis) < 0)
	{
		root_theta = -root_theta;
	}

	bool flipped = std::abs(root_theta) > M_PI * 0.8;
	fallen |= flipped;

	return fallen;
}

