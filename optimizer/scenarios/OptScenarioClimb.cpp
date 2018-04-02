#include "OptScenarioClimb.h"
#include "scenarios/ScenarioClimb.h"

cOptScenarioClimb::cOptScenarioClimb(cOptimizer& optimizer)
	: cOptScenarioTrackMotion(optimizer)
{
}

cOptScenarioClimb::~cOptScenarioClimb()
{
}

std::string cOptScenarioClimb::GetName() const
{
	return "Optimize Climb";
}

void cOptScenarioClimb::BuildScene(int id, std::unique_ptr<cScenarioSimChar>& out_scene)
{
	out_scene = std::unique_ptr<cScenarioTrackMotion>(new cScenarioClimb());
}

double cOptScenarioClimb::CalcCurrErr(const std::unique_ptr<cScenarioSimChar>& scene)
{
	const double pose_err_w = 0.01;
	const double effort_w = 0.00001;
	const double landing_err_w = 5;
	const double com_err_w = 1 * 0;
	const double vel_err_w = 0.1;

	double err = 0;

	const cScenarioClimb* climb_scene = reinterpret_cast<const cScenarioClimb*>(scene.get());
	
	double pose_err = climb_scene->CalcPoseThetaRelErr();
	//double pose_err = climb_scene->CalcPosePosErr();
	double effort = climb_scene->CalcEffort();
	double landing_err = climb_scene->CalcLandingTargetErr();
	double com_err = climb_scene->CalcCOMErr();
	double vel_err = climb_scene->CalcTargetVelErr();

	err = pose_err_w * pose_err
		+ effort_w * effort
		+ landing_err_w * landing_err
		+ com_err_w * com_err
		+ vel_err_w * vel_err;
	
	// hack hack hack
	// time for some hacks
	auto character = climb_scene->GetCharacter();
	int state = character->GetState();

	tVector root_pos = character->GetRootPos();
	tVector shoulder_pos = character->CalcJointPos(13);

	double y_err = 0;
	double y_dist = shoulder_pos[1] - root_pos[1];
	
	if (state < 2)
	{
		y_err = 0.3 - y_dist;
		y_err = std::max(0.0, y_err);
		y_err *= y_err;
		y_err *= 200;
	}

	auto head = character->GetBodyPart(8);
	tVector head_ang_vel = head->GetAngularVelocity();
	double head_vel_err = head_ang_vel.squaredNorm();
	head_vel_err *= 0.01;

	auto ctrl = character->GetController();
	Eigen::VectorXd params;
	ctrl->BuildOptParams(params);

	double front_force_x = params[3];
	double front_force_y = params[4];
	double front_force_er = std::abs(front_force_x) + std::abs(front_force_y);
	front_force_er *= 0.001;

	err += y_err
		+ head_vel_err * 0
		+ front_force_er * 0;

	return err;
}