#pragma once

#include "sim/CtController.h"

class cCtTrackController : public virtual cCtController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cCtTrackController();
	virtual ~cCtTrackController();

	virtual void Init(cSimCharacter* character);
	virtual int GetPoliStateSize() const;
	virtual void SetTargetPoseVel(const Eigen::VectorXd& tar_pose, const Eigen::VectorXd& tar_vel);
	virtual const Eigen::VectorXd& GetTargetPose() const;
	virtual const Eigen::VectorXd& GetTargetVel() const;

protected:
	
	Eigen::VectorXd mTargetPose;
	Eigen::VectorXd mTargetVel;

	virtual void InitTargetPoseVel();
	virtual int GetTargetStateOffset() const;
	virtual int GetTargetStateSize() const;
	virtual void BuildPoliState(Eigen::VectorXd& out_state) const;
	virtual void BuildTargetState(Eigen::VectorXd& out_state) const;
};