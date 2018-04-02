/*
 * BipedController2DParameterized.cpp
 *
 *  Created on: Oct 16, 2016
 *      Author: Glen
 */

#include "BipedController2DParameterized.h"

cBipedController2DParameterized::cBipedController2DParameterized() : cBipedController2DCACLA() //, _rand(14)
{
	// TODO Auto-generated constructor stub

	_target_vel = 3.0;
	_torque_sum = 0.0;
	_num_frames = 0;

}

cBipedController2DParameterized::~cBipedController2DParameterized() {
	// TODO Auto-generated destructor stub
}


void cBipedController2DParameterized::Reset()
{
	// std::cout << "Resetting Dog CACLADQ: " << std::endl;
	bool morphlogyTraining = false;
	/*
	if (morphlogyTraining)
	{
		std::shared_ptr<cWorld> world = this->mChar->GetWorld();

		// this->mChar->Clear();
		delete this->mChar;

		std::shared_ptr<cSimCharacter> mChar1 = std::shared_ptr<cSimDog>(new cSimDog());

		// CreateCharacter(eCharRaptor, mChar1);
		// Want to keep the same controller around but change the simChar.
		cSimCharacter::tParams char_params;
		char_params.mPos = tVector::Zero();
		char_params.mCharFile = "data/characters/dog.txt";
		char_params.mStateFile = "data/states/dog_bound_state3.txt";

		bool succ = mChar1->Init(world, char_params);

		// Json keys
		const std::string gJointsKey = "Joints";
		const std::string gSkeletonKey = "Skeleton";
		const std::string gAttachXKey = "AttachX";

		bool succ1 = true;

		if (char_params.mCharFile != "")
		{
			std::ifstream f_stream(char_params.mCharFile);
			Json::Reader reader;
			Json::Value root;
			succ1 = reader.parse(f_stream, root);
			f_stream.close();

			if (succ1)
			{
				if (root[gSkeletonKey].isNull())
				{
					succ1 = false;
				}
				else
				{
					std::cout << "Controller JSON " << root.toStyledString() << std::endl;
					// Try and change spine a litle
					// double new_length = (root[gSkeletonKey].get(1, 0)[gAttachXKey]).asDouble() + 1.1;
					Json::Value spine0 = (root[gSkeletonKey][gJointsKey].get(1, 0));
					double new_length = (spine0[gAttachXKey]).asDouble() + 1.1;
					// Json::Value & nl(new_length);
					// (root[gSkeletonKey][gJointsKey].get(1, 0))[gAttachXKey].swap(nl);
					// std::cout << "New spine length: " << new_length << std::endl;
					// Json::Value nl(new_length);
					// spine0_attachX.swapPayload(nl);
					// Json::Value nl(new_length);
					// spine0[gAttachXKey].swap(Json::Value(new_length));
					// root["Skeleton"].get(1, 0)["AttachX"].swapPayload(Json::Value(new_length));
					// root["Skeleton"].
					// Json::Value joint_json = joints.get(j, 0);
					std::cout << "Controller JSON " << root.toStyledString() << std::endl;
					succ1 = mChar1->LoadSkeleton(root[gSkeletonKey]);
				}
			}

		}

		if (succ1)
		{
			// mChar1->InitDefaultState();
		}
		// this->Clear();
	}
	*/
	cBipedController2DCACLA::Reset();
}

tVector cBipedController2DParameterized::GetTargetVel() const
{
	return tVector(_target_vel, 0, 0, 0);
}

void cBipedController2DParameterized::Update(double time_step)
{
	
	cBipedController2DCACLA::Update(time_step);
	// Joint torques are calculated in the parent class
}

double cBipedController2DParameterized::CalcReward() const
{
	tVector target_vel = GetTargetVel();

	const double vel_reward_w = 0.8;
	const double torque_reward_w = 0.01;
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
	_target_vel += delta;
	_target_vel = cMathUtil::Clamp(_target_vel, 1.5, 4.5);
	// std::cout << "New target Velocity: " << this->_target_vel << " Torque Reward: " << torque_reward << std::endl;
	// std::cout << "Num Frames: " << _num_frames << std::endl;
	// std::cout << "Reward : " << reward << std::endl;
	_num_frames = 0;
	_torque_sum = 0.0;

	return reward;
}

int cBipedController2DParameterized::GetPoliStateSize() const
{
	int state_size = cTerrainRLCharController::GetPoliStateSize();

	return state_size + 1;
}

void cBipedController2DParameterized::BuildPoliState(Eigen::VectorXd& out_state) const
{
	int state_size = GetPoliStateSize();
	out_state.resize(state_size);

	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	BuildPoliStatePose(pose);
	BuildPoliStateVel(vel);

	int ground_offset = GetPoliStateFeatureOffset(ePoliStateGround);
	int ground_size = GetPoliStateFeatureSize(ePoliStateGround);
	int pose_offset = GetPoliStateFeatureOffset(ePoliStatePose);
	int pose_size = GetPoliStateFeatureSize(ePoliStatePose);
	int vel_offset = GetPoliStateFeatureOffset(ePoliStateVel);
	int vel_size = GetPoliStateFeatureSize(ePoliStateVel);

	out_state.segment(ground_offset, ground_size) = mGroundSamples;
	out_state.segment(pose_offset, pose_size) = pose;
	out_state.segment(vel_offset, vel_size) = vel;
	// Add desired velocity
	out_state(state_size - 1) = this->_target_vel;
}

void cBipedController2DParameterized::getInternalState(Eigen::VectorXd& out) const
{
	// std::cout << "Getting parameterized 2D Biped controller state: " << this->_target_vel << std::endl;
	out.resize(1);
	out(0) = this->_target_vel;
	// return out;
}

void cBipedController2DParameterized::updateInternalState(Eigen::VectorXd& state)
{
	this->_target_vel = state(0);
}

std::string cBipedController2DParameterized::BuildTextInfoStr() const
{
	char char_buffer[64];
	sprintf(char_buffer, "Target Vel: %.2f\n", _target_vel);
	std::string str(char_buffer);
	return str;
}

