#pragma once

#include "anim/MocapStepController.h"

class cMotionFieldStepController : public cMocapStepController {
public:
	
	cMotionFieldStepController();
	virtual ~cMotionFieldStepController();

	virtual void Init(cKinCharacter* character, const std::string& param_file);

private:

	virtual void BuildMotionField();
	virtual int CalcMotionFeatureIdx(int m, double phase) const;

	virtual const tMotionFeatures& GetPhaseMotionFeature(double phase, int motion_id) const;
};
