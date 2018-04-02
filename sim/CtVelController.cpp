#include "CtVelController.h"
#include "sim/SimCharacter.h"

cCtVelController::cCtVelController() : cCtPDController()
{
}

cCtVelController::~cCtVelController()
{
}

void cCtVelController::SetupPDControllers(const std::string& param_file, const tVector& gravity)
{
	cCtPDController::SetupPDControllers(param_file, gravity);

	int num_joints = mPDCtrl.GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		bool valid_pd = mPDCtrl.IsValidPDCtrl(j);
		if (valid_pd)
		{
			mPDCtrl.SetKp(j, 0);
		}
	}
}

void cCtVelController::ApplyAction(const tAction& action)
{
	cCtController::ApplyAction(action);

	int root_size = mChar->GetParamSize(mChar->GetRootID());
	int num_joints = mChar->GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		if (mPDCtrl.IsValidPDCtrl(j))
		{
			int param_offset = mChar->GetParamOffset(j);
			int param_size = mChar->GetParamSize(j);
			param_offset -= root_size;

			Eigen::VectorXd theta_dot = mCurrAction.mParams.segment(param_offset, param_size);
			mPDCtrl.SetTargetVel(j, theta_dot);
		}
	}
}

void cCtVelController::BuildJointActionBounds(int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	cCtCtrlUtil::BuildBoundsVel(joint_mat, joint_id, out_min, out_max);
}

void cCtVelController::BuildJointActionOffsetScale(int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	cCtCtrlUtil::BuildOffsetScaleVel(joint_mat, joint_id, out_offset, out_scale);
}