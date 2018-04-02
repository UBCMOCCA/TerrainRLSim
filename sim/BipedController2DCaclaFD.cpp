/*
 * BipedController2DCaclaFD.cpp
 *
 *  Created on: Nov 18, 2016
 *      Author: Glen B
 */

#include "BipedController2DCaclaFD.h"

cBipedController2DCaclaFD::cBipedController2DCaclaFD() : cTerrainRLCharController(),
cBipedController2D(),
cBaseControllerCaclaFD() {
	// TODO Auto-generated constructor stub
	mExpParams.mBaseActionRate = 0.2;
	mExpParams.mNoise = 0.2;

}

cBipedController2DCaclaFD::~cBipedController2DCaclaFD() {
	// TODO Auto-generated destructor stub
}

void cBipedController2DCaclaFD::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cBaseControllerCaclaFD::Init(character);
	cBipedController2D::Init(character, gravity, param_file);
}

bool cBipedController2DCaclaFD::IsCurrActionCyclic() const
{
	return false;
}

void cBipedController2DCaclaFD::UpdateAction()
{
	cBipedController2D::UpdateAction();
#if defined(ENABLE_DEBUG_VISUALIZATION)
	RecordVal();
#endif // ENABLE_DEBUG_VISUALIZATION
}

void cBipedController2DCaclaFD::Update(double time_step)
{

	cBipedController2D::Update(time_step);

	const cSimBiped2D::eJoint joints[] =
	{
		cSimBiped2D::eJointLeftHip,
		cSimBiped2D::eJointRightHip,
		cSimBiped2D::eJointLeftKnee,
		cSimBiped2D::eJointRightKnee
		// cSimBiped2D::eJointLeftAnkle,
		// cSimBiped2D::eJointRightAnkle
	};
	const int num_joints = sizeof(joints) / sizeof(joints[0]);

	double torque_sum = 0;
	int i = 0;
	for (; i < num_joints; ++i)
		// for (; i < this->mChar->GetNumJoints(); )
	{

		// cJoint& joint = this->mChar->GetJoint(i);
		cJoint& joint = this->mChar->GetJoint(joints[i]);
		tVector torque(0, 0, 0, 0);
		if (joint.IsValid())
		{
			// torque = j.GetTorque();
			// cPDController& pgc = mImpPDCtrl.GetPDCtrl(joints[i]);
			// const cJoint& joint = pgc.GetJoint();
			/*
			tVector axis_rel = joint.GetAxisRel();

			double kp = pgc.GetKd();
			double kd = pgc.GetKp();

			double theta_err = pgc.CalcThetaErr();
			double vel_err = pgc.CalcVelErr();
			double t = kp * theta_err + kd * vel_err;
			torque = axis_rel * t;
			joint.ClampTotalTorque(torque);
			*/
			torque = joint.GetTotalTorque();
			++i;
		}
		// std::cout << "Torque: " << torque[2] << std::endl;
		torque_sum += fabs(torque[2]);

	}

	_torque_sum += (torque_sum / (double)i);
	_num_frames += 1;

}

double cBipedController2DCaclaFD::CalcReward() const
{
	tVector target_vel = GetTargetVel();

	const double vel_reward_w = 0.8;
	const double torque_reward_w = -0.01;
	double vel_reward = 0;
	const double stumble_reward_w = 0.2;
	double stumble_reward = 0;

	bool fallen = mChar->HasFallen();
	if (!fallen)
	{
		double cycle_time = GetPrevCycleTime();
		double avg_vel = mPrevDistTraveled[0] / cycle_time;
		double vel_err = target_vel[0] - avg_vel;
		double vel_gamma = 0.5;
		vel_reward = std::exp(-vel_gamma * vel_err * vel_err);

		double stumble_count = mPrevStumbleCount;
		double avg_stumble = stumble_count /= cycle_time;
		double stumble_gamma = 10;
		stumble_reward = 1.0 / (1 + stumble_gamma * avg_stumble);
	}
	double torque_reward = ((_torque_sum / _num_frames) / 300.0); // scale by max torqe in character file.
	double reward = 0;
	reward += vel_reward_w * vel_reward
		+ stumble_reward_w * stumble_reward
		+ (torque_reward * torque_reward_w);

	double delta = cMathUtil::RandDouble(-0.2, 0.2);
	_num_frames = 0;
	_torque_sum = 0.0;

	return reward;
}



