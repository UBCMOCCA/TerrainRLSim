#include "OptScenarioVelCtrl.h"
#include "scenarios/ScenarioVelCtrl.h"

cOptScenarioVelCtrl::cOptScenarioVelCtrl(cOptimizer& optimizer)
	: cOptScenarioTrackMotion(optimizer)
{
}

cOptScenarioVelCtrl::~cOptScenarioVelCtrl()
{
}

std::string cOptScenarioVelCtrl::GetName() const
{
	return "Optimize Velocity Control";
}

void cOptScenarioVelCtrl::BuildScene(int id, std::shared_ptr<cScenarioSimChar>& out_scene)
{
	out_scene = std::shared_ptr<cScenarioTrackMotion>(new cScenarioVelCtrl());
}

double cOptScenarioVelCtrl::CalcCurrErr(const std::shared_ptr<cScenarioSimChar>& scene)
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
	case cScenarioSimChar::eCharBiped:
	case cScenarioSimChar::eCharBiped2D:
		err = CalcCurrErrBiped(scene);
		break;
	case cScenarioSimChar::eCharHopper:
		err = CalcCurrErrMonopedHopper(scene);
		break;
	case cScenarioSimChar::eCharBiped3D:
		err = CalcCurrErrBiped3D(scene);
		break;
	default:
		assert(false); // unsupported character type
		break;
	}

	return err;
}

double cOptScenarioVelCtrl::CalcCurrErrDog(const std::shared_ptr<cScenarioSimChar>& scene)
{
	const double pose_err_w = 0.02 * 0.01 * 0;
	const double pose_theta_w = 0.2 * 0.01;
	const double vel_err_w = 0.000005 * 0;
	const double root_x_err_w = 0.0075 * 0;
	const double root_y_err_w = 10;
	const double effort_w = 0.00001;
	const double target_vel_w = 10;

	auto track_scene = std::dynamic_pointer_cast<cScenarioTrackMotion>(scene);

	double err = 0;
	double pose_err = track_scene->CalcPosePosErr();
	//double pose_err = track_scene->CalcPoseThetaErr();
	double pose_err_theta = track_scene->CalcPoseThetaRelErr();
	double vel_err = track_scene->CalcVelErr();

	tVector root_delta = track_scene->CalcRootPosErr();
	double root_x_err = root_delta[0] * root_delta[0];
	double root_y_err = root_delta[1] * root_delta[1];

	double target_vel_err = track_scene->CalcTargetVelErr();

	double effort = track_scene->CalcEffort();

	err = pose_err_w * pose_err
		+ pose_theta_w * pose_err_theta
		+ vel_err_w * vel_err
		+ root_x_err_w * root_x_err
		+ root_y_err_w * root_y_err
		+ effort_w * effort
		+ target_vel_w * target_vel_err;

	// hack hack hack
	const auto& character = track_scene->GetCharacter();
	tVector head_vel = character->GetBodyPart(8)->GetLinearVelocity();
	tVector root_vel = character->GetRootVel();
	double key_vel_err = 0.01 * (head_vel[1] * head_vel[1] + root_vel[1] * root_vel[1]);
	const Eigen::VectorXd& pose = character->GetPose();

	tVector toe_pos = character->GetBodyPart(20)->GetPos();
	double toe_err = 0;
	if (character->GetState() == 0)
	{
		tVector root_pos = character->GetRootPos();
		double delta_toe = toe_pos[0] - root_pos[0];
		toe_err = std::min(delta_toe - 0.2, 0.0);
		toe_err *= toe_err;
		toe_err = 10 * toe_err;
	}

	double ankle_pos_err = 0;
	if (character->GetState() <= 1)
	{
		tVector ankle_pos = character->CalcJointPos(19);
		ankle_pos_err = std::min(0.0, ankle_pos[1] - 0.6);
		ankle_pos_err = ankle_pos_err;
		ankle_pos_err *= 0.005;
	}

	double knee_theta_err = 0;
	if (character->GetState() <= 1)
	{
		double knee_theta = pose(character->GetParamOffset(18));
		knee_theta_err = std::abs(knee_theta);
		knee_theta_err *= 0.0005;
	}

	double ankle_theta_err = 0;
	double elbow_theta_err = 0;
	if (character->GetState() == 2)
	{
		double ankle_theta = pose(character->GetParamOffset(19));
		ankle_theta_err = ankle_theta * ankle_theta;
		ankle_theta_err *= 0.0005;

		double elbow_theta = pose(character->GetParamOffset(14));
		elbow_theta_err = elbow_theta * elbow_theta;
		elbow_theta_err *= -0.00002;
	}

	double shoulder_err = 0;
	tVector shoulder_pos = character->GetBodyPart(13)->GetPos();
	shoulder_err = std::min(0.0, shoulder_pos[1] - 0.6);
	shoulder_err *= shoulder_err;
	shoulder_err *= 10;

	tVector root_pos = character->GetRootPos();
	double shoulder_root_err = 0;
	double delta_h = shoulder_pos[1] - root_pos[1];
	shoulder_root_err = std::max(0.0, -(delta_h + 0.1));
	shoulder_root_err *= shoulder_root_err;
	shoulder_root_err *= 5;

	double root_h_err = std::min(0.0, root_pos[1] - 0.5);
	root_h_err *= root_h_err;
	root_h_err *= 5;

	// add hacks
	err += key_vel_err
		+ toe_err
			//+ ankle_pos_err
			//+ knee_theta_err
		+ shoulder_err
		+ root_h_err
		+ shoulder_root_err;
		//+ ankle_theta_err
		//+ elbow_theta_err;

	return err * 0.1;
}


double cOptScenarioVelCtrl::CalcCurrErrRaptor(const std::shared_ptr<cScenarioSimChar>& scene)
{
	const double pose_err_w = 0.02 * 0.01;
	const double vel_err_w = 0.000005 * 0;
	const double root_x_err_w = 0.0075 * 0;
	const double root_y_err_w = 0.001;
	const double effort_w = 0.00001;
	const double target_vel_w = 10;

	auto track_scene = std::dynamic_pointer_cast<cScenarioTrackMotion>(scene);

	double err = 0;
	double pose_err = track_scene->CalcPoseThetaRelErr();
	double vel_err = track_scene->CalcVelErr();

	tVector root_delta = track_scene->CalcRootPosErr();
	double root_x_err = root_delta[0] * root_delta[0];
	double root_y_err = root_delta[1] * root_delta[1];

	double target_vel_err = track_scene->CalcTargetVelErr();

	double effort = track_scene->CalcEffort();

	err = pose_err_w * pose_err
		+ vel_err_w * vel_err
		+ root_x_err_w * root_x_err
		+ root_y_err_w * root_y_err
		+ effort_w * effort
		+ target_vel_w * target_vel_err;

	return err * 0.1;
}


double cOptScenarioVelCtrl::CalcCurrErrBiped(const std::shared_ptr<cScenarioSimChar>& scene)
{
	const double pose_err_w = 0.02 * 0.01;
	const double vel_err_w = 0.000005 * 0;
	const double root_x_err_w = 0.0075 * 0;
	const double root_y_err_w = 0.001;
	const double root_rot_err_w = 1.1;
	const double effort_w = 0.001;
	const double target_vel_w = 10;

	auto track_scene = std::dynamic_pointer_cast<cScenarioTrackMotion>(scene);

	double err = 0;
	double pose_err = track_scene->CalcPoseThetaRelErr();
	double vel_err = track_scene->CalcVelErr();

	const auto& character = track_scene->GetCharacter();

	tQuaternion root_rot = character->GetRootRotation();
	tVector root_axis;
	double root_theta;
	cMathUtil::QuaternionToAxisAngle(root_rot, root_axis, root_theta);
	double root_err = std::fabs(root_theta);

	// tVector root_delta = track_scene->CalcRootPosErr();
	// double root_x_err = root_delta[0] * root_delta[0];
	// double root_y_err = root_delta[1] * root_delta[1];

	double target_vel_err = track_scene->CalcTargetVelErr();

	double effort = track_scene->CalcEffort();

	err = vel_err_w * vel_err
		+ pose_err_w * pose_err
		+ effort_w * effort
		+ root_rot_err_w * root_err
		+ target_vel_w * target_vel_err;

	return err * 0.1;
}

double cOptScenarioVelCtrl::CalcCurrErrBiped3D(const std::shared_ptr<cScenarioSimChar>& scene)
{
	const double pose_err_w = 0.2;
	const double vel_err_w = 0.000005 * 0;
	const double root_x_err_w = 0.0075 * 0;
	const double root_y_err_w = 0.001;
	const double root_rot_err_w = 1.1;
	const double effort_w = 0.001;
	const double target_vel_w = 10;

	auto track_scene = std::dynamic_pointer_cast<cScenarioTrackMotion>(scene);

	double err = 0;
	double pose_err = track_scene->CalcPoseThetaRelErr();
	double vel_err = track_scene->CalcVelErr();

	const auto& character = track_scene->GetCharacter();

	tQuaternion root_rot = character->GetRootRotation();
	tVector root_axis;
	double root_theta;
	cMathUtil::QuaternionToAxisAngle(root_rot, root_axis, root_theta);
	double root_err = std::fabs(root_theta);

	// tVector root_delta = track_scene->CalcRootPosErr();
	// double root_x_err = root_delta[0] * root_delta[0];
	// double root_y_err = root_delta[1] * root_delta[1];

	double target_vel_err = track_scene->CalcTargetVelErr();

	double effort = track_scene->CalcEffort();

	err = 
		// vel_err_w * vel_err
		pose_err_w * pose_err
		+ effort_w * effort
		+ root_rot_err_w * root_err
		// + target_vel_w * target_vel_err
		;

	return err * 0.1;
}

double cOptScenarioVelCtrl::CalcCurrErrMonopedHopper(const std::shared_ptr<cScenarioSimChar>& scene)
{
	const double pose_err_w = 0.02 * 0.01;
	const double vel_err_w = 0.000005 * 0;
	const double root_x_err_w = 0.0075 * 0;
	const double root_y_err_w = 0.001;
	const double effort_w = 0.00001;
	const double target_vel_w = 10;

	auto track_scene = std::dynamic_pointer_cast<cScenarioTrackMotion>(scene);

	double err = 0;
	// double pose_err = track_scene->CalcPoseThetaRelErr();
	double vel_err = track_scene->CalcVelErr();

	// tVector root_delta = track_scene->CalcRootPosErr();
	// double root_x_err = root_delta[0] * root_delta[0];
	// double root_y_err = root_delta[1] * root_delta[1];

	double target_vel_err = track_scene->CalcTargetVelErr();

	double effort = track_scene->CalcEffort();

	err = vel_err_w * vel_err
		+ effort_w * effort
		+ target_vel_w * target_vel_err;

	return err * 0.1;
}

double cOptScenarioVelCtrl::EvalPt(const tPoint& pt, const std::shared_ptr<cScenarioSimChar>& scene)
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
			track_scene->InitTime(init_time);
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
		// std::cout << "Sample: " << i << " objective cost: " << curr_obj_val << std::endl;
		if ( (std::isnan(curr_obj_val) || std::isinf(curr_obj_val) ) )
		{
			curr_obj_val = 30.0;
		}
		std::cout << "Sample: " << i << " objective cost: " << curr_obj_val << std::endl;
		obj_val += curr_obj_val / mEvalSamples;
	}

	//double obj_norm = 1 / (mMaxTime * GetFallPenalty());
	//obj_val *= obj_norm;
	return obj_val;
}