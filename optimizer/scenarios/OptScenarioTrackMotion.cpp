#include "OptScenarioTrackMotion.h"
#include "scenarios/ScenarioTrackMotion.h"
#include "sim/RaptorController.h"

const double gFallPenalty = 10;
const double gStumblePenalty = 10;

cOptScenarioTrackMotion::cOptScenarioTrackMotion(cOptimizer& optimizer)
	: cOptScenarioSimChar(optimizer)
{
	mMaxTime = 10;
	mEvalSamples = 1;
}

cOptScenarioTrackMotion::~cOptScenarioTrackMotion()
{

}

void cOptScenarioTrackMotion::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cOptScenarioSimChar::ParseArgs(parser);

	parser->ParseDouble("scene_max_time", mMaxTime);
	parser->ParseInt("opt_eval_samples", mEvalSamples);
	mEvalSamples = std::abs(mEvalSamples);
}


int cOptScenarioTrackMotion::GetDimensions() const
{
	const auto& controller = GetDefaultController();
	int num_params = controller->GetNumOptParams();
	return num_params;
}

void cOptScenarioTrackMotion::InitPt(tPoint& pt) const
{
	const auto& controller = GetDefaultController();
	int target_ctrl_id = GetTargetCtrlID();
	controller->BuildCtrlOptParams(target_ctrl_id, pt);
}

void cOptScenarioTrackMotion::InitScale(tPoint& scale) const
{
	const auto& controller = GetDefaultController();
	controller->FetchOptParamScale(scale);
}

bool cOptScenarioTrackMotion::IsThreadSafe() const
{
	return true;
}

void cOptScenarioTrackMotion::OutputPt(FILE* f, const tPoint& pt) const
{
	const auto& controller = GetDefaultController();
	controller->OutputOptParams(f, pt);
}

std::string cOptScenarioTrackMotion::GetName() const
{
	return "Optimize Track Motion";
}

void cOptScenarioTrackMotion::BuildScene(int id, std::shared_ptr<cScenarioSimChar>& out_scene)
{
	out_scene = std::shared_ptr<cScenarioTrackMotion>(new cScenarioTrackMotion());
}

double cOptScenarioTrackMotion::CalcCurrErr(const std::shared_ptr<cScenarioSimChar>& scene)
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
	default:
		assert(false); // unsupported character type
		break;
	}

	return err;
}

double cOptScenarioTrackMotion::CalcCurrErrDog(const std::shared_ptr<cScenarioSimChar>& scene)
{
	const double pose_err_w = 0.1;
	const double pose_theta_w = 0.02;
	const double vel_err_w = 0.000005 * 0;
	const double root_x_err_w = 0.0075 * 0;
	const double root_y_err_w = 10;
	const double effort_w = 0.000001;
	const double target_vel_w = 10;

	auto track_scene = std::dynamic_pointer_cast<cScenarioTrackMotion>(scene);

	double err = 0;
	double pose_err = track_scene->CalcPosePosErr();
	//double pose_err = scene->CalcPoseThetaErr();
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
	const auto& character = scene->GetCharacter();
	int state = character->GetState();
	const Eigen::VectorXd& pose = character->GetPose();
	tVector head_vel = character->GetBodyPart(8)->GetLinearVelocity();
	tVector root_vel = character->GetRootVel();
	double key_vel_err = 0.01 * (head_vel[1] * head_vel[1] + 0.25 * root_vel[1] * root_vel[1]);

	tVector toe_pos = character->GetBodyPart(20)->GetPos();
	double toe_err = 0;
	if (state == 0)
	{
		tVector root_pos = character->GetRootPos();
		double delta_toe = toe_pos[0] - root_pos[0];
		toe_err = std::min(delta_toe - 0.2, 0.0);
		toe_err *= toe_err;
		toe_err = 10 * toe_err;
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
	if (state == 2)
	{
		double ankle_theta = pose(character->GetParamOffset(19));
		ankle_theta_err = ankle_theta * ankle_theta;
		ankle_theta_err *= 0.0005;

		double elbow_theta = pose(character->GetParamOffset(14));
		elbow_theta_err = elbow_theta * elbow_theta;
		elbow_theta_err *= -0.00002;
	}

	double shoulder_err = 0;
	tVector shoulder_pos = character->CalcJointPos(13);
	shoulder_err = std::min(0.0, shoulder_pos[1] - 0.6);
	shoulder_err *= shoulder_err;
	shoulder_err *= 20;

	double root_h_err = 0;
	tVector root_pos = character->GetRootPos();
	root_h_err = std::min(0.0, root_pos[1] - 0.55);
	root_h_err *= root_h_err;
	root_h_err *= 10;

	double shoulder_root_err = 0;
	double delta_h = shoulder_pos[1] - root_pos[1];
	//shoulder_root_err = std::max(0.0, -(delta_h + 0.1));
	shoulder_root_err = std::min(0.0, (delta_h + 0.0));
	shoulder_root_err *= shoulder_root_err;
	shoulder_root_err *= 20;

	const auto& ctrl = character->GetController();
	Eigen::VectorXd params;
	ctrl->BuildOptParams(params);

	double hip_target_theta = params[8];
	hip_target_theta = std::min(0.0, hip_target_theta + 0.5);
	double hip_target_err = hip_target_theta * hip_target_theta;
	hip_target_err *= 2;

	double elbow_target_theta = params[19];
	elbow_target_theta = std::min(0.0, elbow_target_theta - 0.2);
	double elbow_target_err = elbow_target_theta * elbow_target_theta;
	elbow_target_err *= 1;

	double hip_theta_err = 0;
	if (state == 1 || state == 2)
	{
		double hip_theta;
		tVector hip_axis;
		character->CalcJointWorldRotation(17, hip_axis, hip_theta);
		hip_theta_err = std::max(0.0, hip_theta - 1.0);
		hip_theta_err *= hip_theta_err;
		hip_theta_err *= 0.5;
	}

	double ankle_target_theta = std::max(params[22], params[28]);
	ankle_target_theta = std::max(0.0, ankle_target_theta - 1.8);
	double ankle_target_err = ankle_target_theta * ankle_target_theta;
	ankle_target_err *= 1;

	double cv = params[0];
	double cv_err = cv - 0.015;
	cv_err *= cv_err;
	cv_err *= 50;

	// force penalty
	double force_err = 0;
	force_err += params[1] * params[1] * 0
		+ params[2] * params[2] * 0
		+ params[3] * params[3]
		+ params[4] * params[4] * 0;
	force_err *= 0.0000005;

	double back_front_err = 0;
	back_front_err = params[1] - params[2];
	back_front_err = std::max(back_front_err, 0.0);
	back_front_err *= back_front_err;
	back_front_err *= 0.00001;

	tVector com = character->CalcCOM();
	double com_h_err = com[1] - 0.5;
	com_h_err = std::abs(std::min(com_h_err, 0.0));
	com_h_err *= com_h_err;
	com_h_err *= 100;

	// add hacks
	err += key_vel_err
		+ toe_err
		+ knee_theta_err * 0
		+ shoulder_err
		+ shoulder_root_err * 0
		+ root_h_err
		+ ankle_theta_err * 0
		+ elbow_theta_err * 0
		+ elbow_target_err * 0
		+ hip_target_err * 0
		+ hip_theta_err * 0
		+ ankle_target_err * 0
		+ cv_err * 0
		+ force_err * 0
		+ back_front_err * 0
		+ com_h_err * 0;

	return err * 0.1;
}

double cOptScenarioTrackMotion::CalcCurrErrRaptor(const std::shared_ptr<cScenarioSimChar>& scene)
{
	const double pose_err_w = 0.2;
	const double vel_err_w = 0.000005 * 0;
	const double root_x_err_w = 0.0075 * 0;
	const double root_y_err_w = 20;
	const double effort_w = 0.000001;
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

	// hacks
	const auto& sim_char = track_scene->GetCharacter();
	const auto& kin_char = track_scene->GetKinCharacter();
	int state = sim_char->GetState();
	std::shared_ptr<cRaptorController> ctrl = std::dynamic_pointer_cast<cRaptorController>(sim_char->GetController());

	tVector sim_root_pos = sim_char->GetRootPos();
	tVector kin_root_pos = kin_char.GetRootPos();

	tVector sim_head_pos = sim_char->CalcJointPos(5);
	tVector kin_head_pos = kin_char.CalcJointPos(5);
	tVector head_delta = kin_head_pos - sim_head_pos;
	double head_err = head_delta[1];
	head_err *= head_err;

	double stance_contact_err = 0;
	if (state == 0)
	{
		int stance_toe_id = ctrl->GetStanceToe();
		bool stance_contact = sim_char->IsInContact(stance_toe_id);
		stance_contact_err = (stance_contact) ? 0 : 100;

		tVector toe_axis;
		double toe_theta;
		sim_char->GetBodyPart(stance_toe_id)->GetRotation(toe_axis, toe_theta);
		stance_contact_err += toe_theta * toe_theta;
	}

	tVector root_axis;
	double root_theta;
	sim_char->GetRootRotation(root_axis, root_theta);
	double root_theta_err = 2 * root_theta * root_theta;

	tVector root_pos = sim_char->GetRootPos();
	double root_h_err = 0.6 - root_pos[1];
	root_h_err = std::max(0.0, root_h_err);
	root_h_err *= root_h_err;
	root_h_err *= 100;

	tVector head_vel = sim_char->GetBodyPart(5)->GetLinearVelocity();
	double head_vel_err = head_vel[1];
	head_vel_err *= head_vel_err;
	head_vel_err *= 0.01;

	double swing_toe_err = 0;
	if (state == 2 || state == 3)
	{
		int swing_toe_id = ctrl->GetSwingToe();
		tVector sim_toe_pos = sim_char->CalcJointPos(swing_toe_id);
		tVector kin_toe_pos = kin_char.CalcJointPos(swing_toe_id);
		sim_toe_pos -= sim_root_pos;
		kin_toe_pos -= kin_root_pos;

		tVector toe_delta = kin_toe_pos - sim_toe_pos;
		swing_toe_err = kin_toe_pos[0] - sim_toe_pos[0];
		swing_toe_err *= swing_toe_err;
		//swing_toe_err *= 10;
	}
	
	double ankle_err = 0;
	int ankle_id = gInvalidIdx;
	//if (state == 0)
	{
		ankle_id = (state == 0) ? ctrl->GetStanceAnkle() : ctrl->GetSwingAnkle();

		tVector kin_ankle_axis;
		double kin_ankle_theta;
		kin_char.CalcJointWorldRotation(ankle_id, kin_ankle_axis, kin_ankle_theta);

		tVector sim_ankle_axis;
		double sim_ankle_theta;
		sim_char->CalcJointWorldRotation(ankle_id, sim_ankle_axis, sim_ankle_theta);

		ankle_err = kin_ankle_theta - sim_ankle_theta;
		ankle_err *= ankle_err;
		ankle_err *= 1;
	}

	err += head_err * 0
		+ stance_contact_err * 0
		+ root_theta_err
		+ root_h_err
		+ head_vel_err * 0
		+ swing_toe_err * 0
		+ ankle_err;
	
	return err * 0.1;
}

double cOptScenarioTrackMotion::CalcCurrErrHopper(const std::shared_ptr<cScenarioSimChar>& scene)
{
	const double pose_err_w = 0.2;
	const double vel_err_w = 0.01;
	const double effort_w = 0.0000001;
	const double target_vel_w = 1;

	auto track_scene = std::dynamic_pointer_cast<cScenarioTrackMotion>(scene);
	const auto& sim_char = track_scene->GetCharacter();

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

	double target_vel_err = track_scene->CalcTargetVelErr();

	double effort = track_scene->CalcEffort();

	err = pose_err_w * pose_err
		+ vel_err_w * vel_err
		//+ effort_w * effort
		+ target_vel_w * target_vel_err;

	return err;
}

double cOptScenarioTrackMotion::GetFallPenalty() const
{
	return gFallPenalty;
}

double cOptScenarioTrackMotion::GetStumblePenalty() const
{
	return gStumblePenalty;
}

const std::shared_ptr<cCharController>& cOptScenarioTrackMotion::GetDefaultController() const
{
	const std::shared_ptr<cScenarioSimChar>& scene = GetDefaultScene();
	const auto& character = scene->GetCharacter();
	const std::shared_ptr<cCharController>& controller = character->GetController();
	return controller;
}

double cOptScenarioTrackMotion::EvalPt(const tPoint& pt, const std::shared_ptr<cScenarioSimChar>& scene)
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
		obj_val += curr_obj_val / mEvalSamples;
	}

	//double obj_norm = 1 / (mMaxTime * GetFallPenalty());
	//obj_val *= obj_norm;
	return obj_val;
}

bool cOptScenarioTrackMotion::CheckTermination(double time, const std::shared_ptr<cScenarioSimChar>& scene) const
{
	return time >= mMaxTime || scene->HasFallen();
}

int cOptScenarioTrackMotion::GetTargetCtrlID() const
{
	auto scene = std::dynamic_pointer_cast<cScenarioTrackMotion>(GetDefaultScene());
	return scene->GetTargetCtrlID();
}

const std::vector<int>& cOptScenarioTrackMotion::GetTargetActions() const
{
	auto scene = std::dynamic_pointer_cast<cScenarioTrackMotion>(GetDefaultScene());
	return scene->GetTargetActions();
}