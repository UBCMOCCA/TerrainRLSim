#pragma once

#include "sim/GroundObstacles3D.h"
#include "sim/SimCharacter.h"

class cGroundObstaclesDynamicCharacters3D : public cGroundObstacles3D
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cGroundObstaclesDynamicCharacters3D();
	virtual ~cGroundObstaclesDynamicCharacters3D();

	virtual void SampleHeightVel(const tVector& pos, double& out_h, tVector& out_vel, 
									bool& out_valid_sample) const;

	virtual void SetChars(const std::vector<std::shared_ptr<cSimCharacter>>& characters);

	virtual void SetChar(const std::shared_ptr<cSimCharacter>& character);

protected:
	// This is the vector of 'other characters'
	std::vector<std::shared_ptr<cSimCharacter>> mChars;

	// This is the main character
	std::shared_ptr<cSimCharacter> mChar;
};
