#include "OptScenarioJump.h"
#include "scenarios/ScenarioJump.h"

cOptScenarioJump::cOptScenarioJump(cOptimizer& optimizer)
	: cOptScenarioTrackMotion(optimizer)
{
}

cOptScenarioJump::~cOptScenarioJump()
{
}

std::string cOptScenarioJump::GetName() const
{
	return "Optimize Jump";
}

void cOptScenarioJump::BuildScene(int id, std::shared_ptr<cScenarioSimChar>& out_scene)
{
	out_scene = std::shared_ptr<cScenarioTrackMotion>(new cScenarioJump());
}

void cOptScenarioJump::UpdateScene(double time_step, const std::shared_ptr<cScenarioSimChar>& scene)
{
	ResetJumpCompleted(scene);
	cOptScenarioTrackMotion::UpdateScene(time_step, scene);
}

void cOptScenarioJump::ResetJumpCompleted(const std::shared_ptr<cScenarioSimChar>& scene)
{
	cScenarioJump* jump_scene = reinterpret_cast<cScenarioJump*>(scene.get());
	jump_scene->ResetJumpCompleted();
}

double cOptScenarioJump::CalcCurrErr(const std::shared_ptr<cScenarioSimChar>& scene)
{
	cScenarioSimChar::eCharType char_type = scene->GetCharType();

	double err = 0;

	switch (char_type)
	{
	case cScenarioSimChar::eCharDog:
		err = CalcCurrErrDog(scene);
		break;
	case cScenarioSimChar::eCharRaptor:
		err = CalcCurrErrRaptor(scene);
		break;
	case cScenarioSimChar::eCharHopper:
		err = CalcCurrErrHopper(scene);
		break;
	case cScenarioSimChar::eCharBiped2D:
		err = CalcCurrErrBiped(scene);
		break;
	default:
		assert(false); // unsupported character type
		break;
	}

	return err;
}

double cOptScenarioJump::CalcCurrErrDog(const std::shared_ptr<cScenarioSimChar>& scene)
{
	const double jump_w = -10;
	const double jump_h_w = -10;
	const double pose_err_w = 0.0001;
	const double effort_w = 0.000001;

	double err = 0;

	const cScenarioJump* jump_scene = reinterpret_cast<const cScenarioJump*>(scene.get());

	//double pose_err = scene->CalcPosePosErr();
	double pose_err = jump_scene->CalcPoseThetaRelErr();
	double effort = jump_scene->CalcEffort();
	double jump_reward = CalcJumpReward(scene);
	double height_reward = jump_scene->CalcJumpHeightReward();

	if (!jump_scene->Jumping())
	{
		//pose_err = 0;
		effort = 0;
	}

	err = pose_err_w * pose_err
		+ effort_w * effort
		+ jump_w * jump_reward
		+ jump_h_w * height_reward;


	// hacks
	const auto& character = scene->GetCharacter();
	int state = character->GetState();

	tVector root_pos = character->GetRootPos();
	tVector torso_pos = character->GetBodyPart(5)->GetPos();

	const Eigen::VectorXd& pose = character->GetPose();

	if (jump_scene->Jumping())
	{
		const double tilt_err_w = 500;
		double tilt_err = 0;
		if (state == 1 || state == 2)
		{
			double delta_h = torso_pos[1] - root_pos[1];
			if (delta_h < 0)
			{
				tilt_err = std::min(0.0, delta_h + 0.3);
			}
			
			//tilt_err = delta_h + 0.3;
			tilt_err *= tilt_err;
		}

		const double hip_err_w = 20;
		double hip_err = 0;
		if (state == 1)
		{
			double hip_theta;
			tVector hip_axis;
			character->CalcJointWorldRotation(17, hip_axis, hip_theta);
			double hip_err = std::min(0.0, hip_theta - 1.0);
			hip_err *= hip_err;
		}

		double knee_err = 0;
		if (state == 1)
		{
			double knee_theta = pose(character->GetParamOffset(18));
			double knee_err = std::max(0.0, M_PI - std::abs(knee_theta) - 0.1);
			knee_err *= knee_err;
		}
		knee_err *= 0.1;

		double elbow_theta_err = 0;
		if (state == 1
			|| state == 2)
		{
			double elbow_theta = pose(character->GetParamOffset(14));
			elbow_theta = std::min(elbow_theta - 0.2, 0.0);
			//elbow_theta = std::max(0.0, elbow_theta - 0.2);
			//elbow_theta_err = 1 / (1 + 100 * elbow_theta * elbow_theta);

			elbow_theta *= elbow_theta;
			elbow_theta_err = elbow_theta;
			elbow_theta_err *= 0.05;
		}

		double shoulder_theta_err = 0;
		if (state == 1)
		{
			double shoulder_theta = pose(character->GetParamOffset(13));
			//shoulder_theta = std::max(0.0, shoulder_theta - 0.6);
			//shoulder_theta_err = 1 / (1 + 20 * shoulder_theta * shoulder_theta);
			
			shoulder_theta -= 0.6;
			shoulder_theta = std::max(0.0, shoulder_theta);
			shoulder_theta *= shoulder_theta;
			shoulder_theta_err *= 2;
			//shoulder_theta_err *= 0.1;
		}

		const auto& ctrl = character->GetController();
		Eigen::VectorXd params;
		ctrl->BuildCtrlOptParams(2, params);

		double hip_target_theta = params[8];
		hip_target_theta = std::min(0.0, hip_target_theta + 1.0);
		double hip_target_err = hip_target_theta * hip_target_theta;
		hip_target_err *= 10;

		double hip_target_theta1 = params[14];
		hip_target_theta1 = std::min(0.0, hip_target_theta1 + 0.4);
		double hip_target_err1 = hip_target_theta1 * hip_target_theta1;
		hip_target_err1 *= 10;

		double knee_target_theta = params[9];
		knee_target_theta = std::max(0.0, knee_target_theta);
		double knee_target_err = knee_target_theta * knee_target_theta;
		knee_target_err *= 0.1 * 0;

		//double ankle_target_theta = std::min(params[10], params[16]);
		double ankle_target_theta = params[16];
		ankle_target_theta = std::min(0.0, ankle_target_theta - 1.2);
		double ankle_target_err = ankle_target_theta * ankle_target_theta;
		ankle_target_err *= 0.1;

		double elbow_target_theta = params[13];
		elbow_target_theta = std::min(0.0, elbow_target_theta - 0.2);
		double elbow_target_err = elbow_target_theta * elbow_target_theta;
		elbow_target_err *= 2;

		double cv = params[0];
		double cv_err = cv - 0.025;
		cv_err *= cv_err;
		cv_err *= 200;

		double contact_err = 0;
		if (character->IsInContact(13)
			|| character->IsInContact(14)
			|| character->IsInContact(15)
			|| character->IsInContact(17)
			|| character->IsInContact(18)
			|| character->IsInContact(19))
		{
			contact_err = 100;
		}

		tVector toe_pos = character->GetBodyPart(20)->GetPos();
		double toe_err = 0;

		if (jump_scene->Jumping())
		{
			if (state == 2 || state == 3)
			{
				tVector root_pos = character->GetRootPos();
				double delta_toe = toe_pos[0] - root_pos[0];
				toe_err = std::max(delta_toe, 0.0);
				toe_err *= toe_err;
				toe_err = 1000 * toe_err;
			}
		}

		// add hacks
		err += 0.2 * (tilt_err_w * tilt_err
			+ hip_err_w * hip_err * 0
			+ elbow_theta_err * 0
			+ shoulder_theta_err
			+ knee_err * 0
			+ cv_err * 0
			+ hip_target_err * 0
			+ hip_target_err1 * 0
			+ knee_target_err * 0
			+ ankle_target_err * 0
			+ elbow_target_err * 0
			+ toe_err);
	}

	return err;
}

double cOptScenarioJump::CalcCurrErrRaptor(const std::shared_ptr<cScenarioSimChar>& scene)
{
	const double jump_w = -10;
	const double jump_h_w = -10;
	const double pose_err_w = 0.005;
	const double effort_w = 0.000001;

	double err = 0;

	const cScenarioJump* jump_scene = reinterpret_cast<const cScenarioJump*>(scene.get());

	double pose_err = jump_scene->CalcPoseThetaRelErr();
	double effort = jump_scene->CalcEffort();
	double jump_reward = CalcJumpReward(scene);
	double height_reward = jump_scene->CalcJumpHeightReward();

	if (!jump_scene->Jumping())
	{
		//pose_err = 0;
		effort = 0;
	}

	err = pose_err_w * pose_err
		+ effort_w * effort
		+ jump_w * jump_reward
		+ jump_h_w * height_reward;
	
	return err;
}


double cOptScenarioJump::CalcCurrErrHopper(const std::shared_ptr<cScenarioSimChar>& scene)
{
	const double jump_w = -10;
	const double jump_h_w = -10;
	const double pose_err_w = 0.002;
	const double vel_err_w = 0.001 * 0;
	const double effort_w = 0.00000001;

	const auto& sim_char = scene->GetCharacter();
	const cScenarioJump* jump_scene = reinterpret_cast<const cScenarioJump*>(scene.get());

	double err = 0;

	tVector root_axis = tVector::Zero();
	double root_theta = 0;
	sim_char->GetRootRotation(root_axis, root_theta);

	tVector leg_axis = tVector::Zero();
	double leg_theta = 0;
	sim_char->GetBodyPart(1)->GetRotation(leg_axis, leg_theta);

	double pose_err = root_theta * root_theta
					+ 0.1 * leg_theta * leg_theta;

	tVector root_ang_vel = sim_char->GetRootAngVel();
	double vel_err = root_ang_vel.squaredNorm();

	double target_vel_err = jump_scene->CalcTargetVelErr();

	double effort = jump_scene->CalcEffort();

	double jump_reward = CalcJumpReward(scene);
	double height_reward = jump_scene->CalcJumpHeightReward();

	err = pose_err_w * pose_err
		+ vel_err_w * vel_err
		//+ effort_w * effort
		+ jump_w * jump_reward
		+ jump_h_w * height_reward;

	return err;
}


double cOptScenarioJump::CalcCurrErrBiped(const std::shared_ptr<cScenarioSimChar>& scene)
{
	const double jump_w = -1.0;
	const double torso_pitch_w = 1.0;
	const double jump_h_w = -10.0;
	const double pose_err_w = 0.005;
	const double effort_w = 0.000001;

	double err = 0;

	const cScenarioJump* jump_scene = reinterpret_cast<const cScenarioJump*>(scene.get());
	const auto& sim_char = scene->GetCharacter();

	double root_theta = 0;
	tVector root_axis = tVector::Zero();
	sim_char->GetRootRotation(root_axis, root_theta);

	double pose_err = jump_scene->CalcPoseThetaRelErr();
	double effort = jump_scene->CalcEffort();
	double jump_reward = CalcJumpReward(scene);
	double height_reward = jump_scene->CalcJumpHeightReward();

	if (!jump_scene->Jumping())
	{
		//pose_err = 0;
		effort = 0;
	}

	err = pose_err_w * pose_err
		+ effort_w * effort
		+ jump_w * jump_reward
		+ jump_h_w * height_reward
		+ ((root_theta*root_theta)* torso_pitch_w);

	return err;
}

double cOptScenarioJump::CalcJumpReward(const std::shared_ptr<cScenarioSimChar>& scene) const
{
	const cScenarioJump* jump_scene = reinterpret_cast<const cScenarioJump*>(scene.get());
	double jump_reward = 0;
	if (jump_scene->JumpCompleted())
	{
		tVector jump_dist = jump_scene->GetJumpDist();
		jump_reward = jump_dist[0];
	}
	return jump_reward / gTimeStep;
}

bool cOptScenarioJump::CheckTermination(double time, const std::shared_ptr<cScenarioSimChar>& scene) const
{
	return cOptScenarioTrackMotion::CheckTermination(time, scene);
	/*
	bool terminate = false;

	const cScenarioJump* jump_scene = reinterpret_cast<const cScenarioJump*>(scene.get());
	const auto& character = scene->GetCharacter();
	bool fallen = character->HasFallen();
	terminate = fallen;

	bool jumping = jump_scene->Jumping();
	if (jumping)
	{
		bool stumble = character->HasStumbled();
		terminate |= stumble;
	}
	return terminate;
	*/
}

double cOptScenarioJump::EvalPt(const tPoint& pt, const std::shared_ptr<cScenarioSimChar>& scene)
{
	const double obj_scale = 0.1;

	auto track_scene = std::dynamic_pointer_cast<cScenarioTrackMotion>(scene);

	const auto& kin_char = track_scene->GetKinCharacter();
	const auto& sim_char = track_scene->GetCharacter();
	double motion_dur = kin_char.GetMotionDuration();
	double time_min = 0;
	double time_max = motion_dur;

	double obj_val = 0;
	for (int i = 0; i < mEvalSamples; ++i)
	{
		track_scene->Reset();

		if (mEvalSamples > 1)
		{
			double init_time = time_min;
			init_time = time_min + (time_max - time_min) * static_cast<double>(i) / mEvalSamples;
			// hack hack hack
			// track_scene->InitTime(init_time);
		}

		int target_ctrl_id = GetTargetCtrlID();
		auto& controller = sim_char->GetController();
		controller->SetCtrlOptParams(target_ctrl_id, pt);

		double curr_time = 0;

		double curr_obj_val = 0;
		while (!CheckTermination(curr_time, scene))
		{
			UpdateScene(gTimeStep, scene);
			curr_time += gTimeStep;

			double err = CalcCurrErr(scene);

			bool stumble = track_scene->HasStumbled();
			if (stumble)
			{
				err += GetStumblePenalty();
			}

			curr_obj_val += gTimeStep * err;

			bool fall = track_scene->HasFallen();
			if (fall)
			{
				const double fall_time_penalty = GetFallPenalty();
				double time_remain = mMaxTime - curr_time;
				curr_obj_val += time_remain * fall_time_penalty;
			}
		}

		curr_obj_val *= obj_scale;
		obj_val += curr_obj_val / mEvalSamples;
	}

	//double obj_norm = 1 / (mMaxTime * GetFallPenalty());
	//obj_val *= obj_norm;
	return obj_val;
}
