#include "anim/MotionFieldStepController.h"

const int gMotionFieldPhaseSamples = 16;

cMotionFieldStepController::cMotionFieldStepController()
{
}

cMotionFieldStepController::~cMotionFieldStepController()
{
}

void cMotionFieldStepController::Init(cKinCharacter* character, const std::string& param_file)
{
	cMocapStepController::Init(character, param_file);
	BuildMotionField();
}

void cMotionFieldStepController::BuildMotionField()
{
	int num_motions = GetNumMotions();
	mMotionFeatures.resize(num_motions * gMotionFieldPhaseSamples);
	for (int m = 0; m < num_motions; ++m)
	{
		const cMotion& curr_motion = mMotions[m];

		for (int i = 0; i < gMotionFieldPhaseSamples; ++i)
		{
			double curr_phase = i / (gMotionFieldPhaseSamples - 1.0);
			tMotionFeatures curr_features;
			ExtractMotionFeatures(curr_motion, curr_phase, curr_features);

			int feature_idx = CalcMotionFeatureIdx(m, curr_phase);
			mMotionFeatures[feature_idx] = curr_features;
		}
	}
}

int cMotionFieldStepController::CalcMotionFeatureIdx(int m, double phase) const
{
	int phase_idx = static_cast<int>(phase * (gMotionFieldPhaseSamples - 1));
	return m * gMotionFieldPhaseSamples + phase_idx;
}

const cMotionFieldStepController::tMotionFeatures& cMotionFieldStepController::GetPhaseMotionFeature(double phase, int motion_id) const
{
	double step_phase = CalcStepPhase(phase);
	int feature_idx = CalcMotionFeatureIdx(motion_id, phase);
	return mMotionFeatures[feature_idx];
}