#include "sim/GroundObstaclesDynamicCharacters3D.h"

const double gDefaultHeight = 0;

cGroundObstaclesDynamicCharacters3D::cGroundObstaclesDynamicCharacters3D()
{
}

cGroundObstaclesDynamicCharacters3D::~cGroundObstaclesDynamicCharacters3D()
{
}

void cGroundObstaclesDynamicCharacters3D::SetChars(const std::vector<std::shared_ptr<cSimCharacter>>& characters)
{
	mChars = characters;
}

void cGroundObstaclesDynamicCharacters3D::SetChar(const std::shared_ptr<cSimCharacter>& character)
{
	mChar = character;
}

void cGroundObstaclesDynamicCharacters3D::SampleHeightVel(const tVector& pos, double& out_h, tVector& out_vel,
													bool& out_valid_sample) const
{
	out_h = gDefaultHeight;
	out_vel = tVector::Zero();
	bool foundIntersectedChar = false;

	// Check to see if this sample point contains any of the other characters
	for(std::vector<std::shared_ptr<cSimCharacter>>::const_iterator it = mChars.begin(); it != mChars.end(); ++it) 
	{
		tVector aabb_min;
		tVector aabb_max;
		(*it)->CalcAABB(aabb_min, aabb_max);
		
		if (cMathUtil::ContainsAABBXZ(pos, aabb_min, aabb_max))
		{
			out_h = aabb_max[1];
			out_vel = (*it)->GetRootVel();
			foundIntersectedChar = true;
			break;
		}
	}

	// If we havent found one intersection amongtst the additional character, then check the default one as well
	if (!foundIntersectedChar)
	{
		tVector aabb_min;
		tVector aabb_max;
		mChar->CalcAABB(aabb_min, aabb_max);
		
		if (cMathUtil::ContainsAABBXZ(pos, aabb_min, aabb_max))
		{
			out_h = aabb_max[1];
			out_vel = mChar->GetRootVel();
		}
	}

	out_valid_sample = true;
}