/*
 * SimAdapter.cpp
 *
 *  Created on: 2017-01-20
 *      Author: gberseth
 */

#include "SimAdapter.h"
#include "Main.h"
#include "sim/TerrainRLCharController.h"
#include "sim/WaypointController.h"
#include "scenarios/ScenarioExp.h"
#include "scenarios/DrawScenarioImitateEval.h"
#include "scenarios/ScenarioImitate.h"
#include "scenarios/ScenarioImitateEval.h"
#include "scenarios/ScenarioImitateStepEval.h"
#include "scenarios/ScenarioExp.h"
#include "scenarios/ScenarioHikeEval.h"
#include "scenarios/ScenarioSoccerEval.h"


cSimAdapter::cSimAdapter(std::vector<std::string> args) {
	// TODO Auto-generated constructor stub
	_lastControllerState=0;
	_render=false;
	_gScenario = nullptr;
	_args = args;
	this->_relativePath = "";
}

cSimAdapter::~cSimAdapter() {
	// TODO Auto-generated destructor stub
}

void cSimAdapter::setRender(bool shouldRender)
{
	this->_render=shouldRender;
}

void cSimAdapter::init()
{
	// testBVHReader();
    char** cstrings = new char*[_args.size()];
    for(size_t i = 0; i < _args.size(); ++i)
    {
        cstrings[i] = new char[_args[i].size() + 1];
        std::strcpy(cstrings[i], _args[i].c_str());
    }

	int argc = _args.size();
	// char* argv[] = cstrings;

 	gArgc = argc;
	gArgv = cstrings;
	ParseArgs(gArgc, gArgv);

	std::string scenario_name = "";
	gArgParser->ParseString("scenario", scenario_name);

	if (scenario_name == "hike_eval"
					)
	{
		gCameraPosition = tVector(0, 50, 100, 0);
	}

	InitCaffe();

	if (_render)
	{
		glutInit(&gArgc, gArgv);
		glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
		glutInitWindowSize(gWinWidth, gWinHeight);
		glutCreateWindow("Terrain RL");

		InitOpenGl();
	}
	// SetupScenario();
	// ClearScenario();

	// if (scenario_name == "sim_char")
	if ( _render )
	{
		InitCamera();
		ClearScenario();

		if (scenario_name == "imitate_eval")
		{
			std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioImitateEval>(new cDrawScenarioImitateEval(gCamera));
			// std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioSimChar>(new cDrawScenarioSimChar(gCamera));
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__->GetScene());
			this->_gScenario = scenario__;
			if (this->_gScenario != NULL)
			{
				auto sim_char_scene = std::dynamic_pointer_cast<cDrawScenarioTerrainRL>(scenario__);
				if (sim_char_scene != nullptr)
				{
					sim_char_scene->SetOutputTex(gIntermediateFrameBuffer);
				}

			}
			gScenario = std::shared_ptr<cDrawScenario>(scenario__);
			this->_gScenario = gScenario;
		}
		else if (scenario_name == "sim_char")
		{
			std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioSimChar>(new cDrawScenarioSimChar(gCamera));
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__->GetScene());
			this->_gScenario = scenario__;
			if (this->_gScenario != NULL)
			{
				auto sim_char_scene = std::dynamic_pointer_cast<cDrawScenarioTerrainRL>(scenario__);
				if (sim_char_scene != nullptr)
				{
					sim_char_scene->SetOutputTex(gIntermediateFrameBuffer);
				}

			}
			gScenario = std::shared_ptr<cDrawScenario>(scenario__);
			this->_gScenario = gScenario;
		}
		else if (scenario_name == "imitate_step_eval")
		{
			std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioImitateStepEval>(new cDrawScenarioImitateStepEval(gCamera));
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__->GetScene());
			this->_gScenario = scenario__;
			if (this->_gScenario != NULL)
			{
				auto sim_char_scene = std::dynamic_pointer_cast<cDrawScenarioTerrainRL>(scenario__);
				if (sim_char_scene != nullptr)
				{
					sim_char_scene->SetOutputTex(gIntermediateFrameBuffer);
				}

			}
			gScenario = std::shared_ptr<cDrawScenario>(scenario__);
			this->_gScenario = gScenario;
		}
		else if (scenario_name == "hike_eval")
		{
			gCameraPosition = tVector(0, 30, 30, 0);

			std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioHikeEval>(new cDrawScenarioHikeEval(gCamera));
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__->GetScene());
			this->_gScenario = scenario__;
			if (this->_gScenario != NULL)
			{
				auto sim_char_scene = std::dynamic_pointer_cast<cDrawScenarioTerrainRL>(scenario__);
				if (sim_char_scene != nullptr)
				{
					sim_char_scene->SetOutputTex(gIntermediateFrameBuffer);
				}

			}
			gScenario = std::shared_ptr<cDrawScenario>(scenario__);
			this->_gScenario = gScenario;
		}
		else if (scenario_name == "soccer_eval")
		{
			gCameraPosition = tVector(0, 100, 100, 0);
			std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioSoccerEval>(new cDrawScenarioSoccerEval(gCamera));
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__->GetScene());
			this->_gScenario = scenario__;
			if (this->_gScenario != NULL)
			{
				auto sim_char_scene = std::dynamic_pointer_cast<cDrawScenarioTerrainRL>(scenario__);
				if (sim_char_scene != nullptr)
				{
					sim_char_scene->SetOutputTex(gIntermediateFrameBuffer);
				}

			}
			gScenario = std::shared_ptr<cDrawScenario>(scenario__);
			this->_gScenario = gScenario;
		}
		else
		{
			std::cerr << "scenario not recognized: " << scenario_name << std::endl;
			exit(-1);
		}
	}
	// else if (scenario_name == "sim_char_render")
	else
	{
		ClearScenario();
		if (scenario_name == "imitate_eval")
		{
			// std::shared_ptr<cScenarioImitate> scenario__ = std::shared_ptr<cScenarioImitate>(new cScenarioImitate());
			std::shared_ptr<cScenarioSimChar> scenario__ = std::shared_ptr<cScenarioSimChar>(new cScenarioImitateEval() );
			this->_scene = std::dynamic_pointer_cast<cScenarioSimChar>(scenario__);
			// gScenario = std::shared_ptr<cScenario>(scenario__);
			this->_gScenario = scenario__ ;
		}
		else if (scenario_name == "sim_char")
		{
			std::shared_ptr<cScenarioSimChar> scenario__ = std::shared_ptr<cScenarioSimChar>(new cScenarioSimChar());
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__);
			// gScenario = std::shared_ptr<cScenario>(scenario__);
			this->_gScenario = scenario__ ;
		}
		else if (scenario_name == "imitate_step_eval")
		{
			std::shared_ptr<cScenarioSimChar> scenario__ = std::shared_ptr<cScenarioImitateStepEval>(new cScenarioImitateStepEval());
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__);
			// gScenario = std::shared_ptr<cScenario>(scenario__);
			this->_gScenario = scenario__ ;
		}
		else if (scenario_name == "hike_eval")
		{
			gCameraPosition = tVector(0, 30, 30, 0);
			std::shared_ptr<cScenarioSimChar> scenario__ = std::shared_ptr<cScenarioHikeEval>(new cScenarioHikeEval());
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__);
			// gScenario = std::shared_ptr<cScenario>(scenario__);
			this->_gScenario = scenario__ ;
		}
		else if (scenario_name == "soccer_eval")
		{
			gCameraPosition = tVector(0, 30, 30, 0);
			std::shared_ptr<cScenarioSimChar> scenario__ = std::shared_ptr<cScenarioSoccerEval>(new cScenarioSoccerEval());
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__);
			this->_gScenario = scenario__ ;
		}
		else
		{
			std::cout << "Scenario type not yet supported by adapter: " << scenario_name << std::endl;

		}
		// gScenario = this->_gScenario;
	}

	this->_gScenario->ParseArgs(gArgParser);
	this->_gScenario->Init();
	printf("Loaded scenario: %s\n", this->_gScenario->GetName().c_str());
	if ( _render )
	{
		this->_scene = std::shared_ptr<cScenarioSimChar>(std::dynamic_pointer_cast<cDrawScenarioSimChar>(this->_gScenario)->GetScene());
		Reshape(gWinWidth, gWinHeight);
		glutDisplayFunc(Display);
		glutReshapeFunc(Reshape);
		glutKeyboardFunc(Keyboard);
		glutMouseFunc(MouseClick);
		glutMotionFunc(MouseMove);
		// glutTimerFunc(gDisplayAnimTime, Animate, 0);
	}

	InitTime();
	// glutMainLoop();

	for(size_t i = 0; i < _args.size(); ++i)
	{
		// delete[] cstrings[i];
	}
	// return EXIT_SUCCESS;
}

void cSimAdapter::initEpoch()
{

	this->_gScenario->Reset();
	gForceClear = true;
}

void cSimAdapter::update()
{

	if (gAnimate)
	{
		if ( _render )
		{
			int num_steps = GetNumTimeSteps();
			int current_time = glutGet(GLUT_ELAPSED_TIME);
			int elapsedTime = current_time - gPrevTime;

			gPrevTime = current_time;

			double timestep = (gPlaybackSpeed < 0) ? -gAnimStep : gAnimStep;
			// elapsedTime = int( 1000 * timestep );


			if (this->_gScenario != NULL)
			{

				for (size_t i = 0; i < num_steps; ++i)
				{
					this->_gScenario->Update(timestep);
				}
			}
			/*
			double timestep = ((gPlaybackSpeed < 0) ? -gAnimStep : gAnimStep);
			timestep = timestep;
			for (int i = 0; i < num_steps; ++i)
			{
				Update(timestep);
			}
			 */
			int update_dur = glutGet(GLUT_ELAPSED_TIME) - current_time;

			// glutPostRedisplay();
			gUpdatesPerSec = (num_steps / (elapsedTime * 0.001));
			int timer_step = CalcDisplayAnimTime();
			timer_step -= update_dur;
			timer_step = std::max(timer_step, 0);

			// glutTimerFunc(timer_step, Animate, 0);
		}
		else
		{
			int num_steps = GetNumTimeSteps();

			double timestep = (gPlaybackSpeed < 0) ? -gAnimStep : gAnimStep;
			timestep = timestep;

			if (this->_gScenario != NULL)
			{

				for (size_t i = 0; i < num_steps; ++i)
				{
					this->_gScenario->Update(timestep);
				}
			}
		}
	}

	if (this->_gScenario != nullptr)
	{
		if (this->_gScenario->IsDone())
		{
			Shutdown();
		}
	}
	/*
	if ( _render )
	{
		Display();
	}
	*/
}

void cSimAdapter::display()
{
	if ( _render )
	{
		Display();
	}
}

void cSimAdapter::finish()
{
	if (this->_gScenario != nullptr)
	{
		if (this->_gScenario->IsDone())
		{
			Shutdown();
		}
	}
}

double cSimAdapter::calcReward()
{

	// std::shared_ptr<cScenarioExp> tmp_scenario = std::static_pointer_cast< cScenarioExp>(this->_gScenario);
	double reward_ = this->_scene->CalcReward();
	/*
	std::cout << "Reward: " << r << std::endl;
	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	// char_ = scene->getChar();
	std::shared_ptr<cTerrainRLCharController> controller =  std::static_pointer_cast< cTerrainRLCharController >(char_->GetController());
	double reward_ = controller->CalcReward();
	*/
	return reward_;
}

std::vector<double> cSimAdapter::calcRewards()
{

	// std::shared_ptr<cScenarioExp> tmp_scenario = std::static_pointer_cast< cScenarioExp>(this->_gScenario);
	std::vector<double> reward_ = this->_scene->CalcRewards();
	/*
	std::cout << "Reward: " << r << std::endl;
	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	// char_ = scene->getChar();
	std::shared_ptr<cTerrainRLCharController> controller =  std::static_pointer_cast< cTerrainRLCharController >(char_->GetController());
	double reward_ = controller->CalcReward();
	*/
	return reward_;
}

double cSimAdapter::calcVelocity()
{
	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	return char_->GetRootVel()(0);
}

bool cSimAdapter::hasStumbled()
{
	// const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	return this->_scene->HasStumbled();
}

double cSimAdapter::jointTorque()
{
	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	size_t num_joints = char_->GetNumJoints();
	double torque_sum = 0;
	for (size_t i=0; i < num_joints; ++i)
	{

		cJoint& joint = char_->GetJoint(i);
		tVector torque(0, 0, 0, 0);
		if (joint.IsValid())
		{
			torque = joint.GetTotalTorque();
		}
		// std::cout << "Torque: " << torque[2] << std::endl;
		/// Only for 2D for now...
		torque_sum += fabs(torque[2]);

	}
	/// divided by max torque (300)
	torque_sum = (torque_sum / (double)num_joints) / 300.0;
	return torque_sum;
}

double cSimAdapter::updateAction(std::vector<double> act)
{
	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	// char_ = scene->getChar();
	std::shared_ptr<cTerrainRLCharController> controller =  std::static_pointer_cast< cTerrainRLCharController >(char_->GetController());
	// std::cout << "Action length: " << controller->GetPoliActionSize() << std::endl;
	// std::cout << "Num controller params: " << controller->GetNumParams() << std::endl;
	// ASSERT(controller->GetPoliActionSize() == act.size(),  "Action length (" << act.size() << ") incorrect, should be " << controller->GetPoliActionSize());

	cTerrainRLCharController::tAction action;
	action.mID;
	action.mParams = Eigen::VectorXd::Zero(act.size());
	for (size_t i=0; i < act.size(); i++)
	{
		action.mParams(i) = act[i];
	}

	controller->ApplyAction(action);
	return 0.0;
}

double cSimAdapter::updateLLCAction(std::vector<double> act)
{
	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	// char_ = scene->getChar();
	auto controller = std::dynamic_pointer_cast<cWaypointController>(char_->GetController());
		// std::shared_ptr<cWaypointController> controller =  std::static_pointer_cast< cWaypointController >(char_->GetController());
	if (controller != nullptr)
	{
		std::shared_ptr<cBipedStepController3D> llc = controller->GetLLC();

		cTerrainRLCharController::tAction action;
		action.mID;
		action.mParams = Eigen::VectorXd::Zero(act.size());
		for (size_t i=0; i < act.size(); i++)
		{
			action.mParams(i) = act[i];
		}

		llc->ApplyAction(action);
	}
	// std::cout << "Action length: " << controller->GetPoliActionSize() << std::endl;
	// std::cout << "Num controller params: " << controller->GetNumParams() << std::endl;
	// ASSERT(controller->GetPoliActionSize() == act.size(),  "Action length (" << act.size() << ") incorrect, should be " << controller->GetPoliActionSize());

	return 0.0;
}

void cSimAdapter::act(std::vector<double> act)
{

	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	// char_ = scene->getChar();
	std::shared_ptr<cTerrainRLCharController> controller =  std::static_pointer_cast< cTerrainRLCharController >(char_->GetController());
	// std::cout << "Action length: " << controller->GetPoliActionSize() << std::endl;
	// std::cout << "Num controller params: " << controller->GetNumParams() << std::endl;
	// ASSERT(controller->GetPoliActionSize() == act.size(),  "Action length (" << act.size() << ") incorrect, should be " << controller->GetPoliActionSize());

	cTerrainRLCharController::tAction action;
	action.mID;
	action.mParams = Eigen::VectorXd::Zero(act.size());
	for (size_t i=0; i < act.size(); i++)
	{
		action.mParams(i) = act[i];
	}

	controller->ApplyAction(action);

}

std::vector<double> cSimAdapter::getState() const
{
	Eigen::VectorXd state;
	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	// char_ = scene->getChar();
	std::shared_ptr<cTerrainRLCharController> controller =  std::static_pointer_cast< cTerrainRLCharController >(char_->GetController());
	// const std::shared_ptr<cTerrainRLCharController>& controller =  static_cast< const std::shared_ptr<cTerrainRLCharController>& >(char_->GetController());
	// controller = char->getCOntroller()
	controller->ParseGround();
	controller->BuildPoliState(state);
	// std::cout << state.transpose() << std::endl;

	std::vector<double> out(state.data(), state.data() + state.rows() * state.cols());
	return out;

}

std::vector<double> cSimAdapter::getLLCState()
{

	Eigen::VectorXd state;
	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	// char_ = scene->getChar();
	auto controller = std::dynamic_pointer_cast<cWaypointController>(char_->GetController());
	// std::shared_ptr<cWaypointController> controller =  std::static_pointer_cast< cWaypointController >(char_->GetController());
	if (controller != nullptr)
	{
		std::shared_ptr<cBipedStepController3D> llc = controller->GetLLC();
		llc->BuildPoliState(state);
	}
	// const std::shared_ptr<cTerrainRLCharController>& controller =  static_cast< const std::shared_ptr<cTerrainRLCharController>& >(char_->GetController());
	// controller = char->getCOntroller()
	// controller->ParseGround();
	// controller->BuildPoliState(state);
	// std::cout << state.transpose() << std::endl;

	std::vector<double> out(state.data(), state.data() + state.rows() * state.cols());
	return out;
}

bool cSimAdapter::endOfEpoch()
{

	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();

	// std::cout << "Fallen: " << char_->HasFallen() << std::endl;
	if (char_->HasFallen() || char_->HasExploded())
	{
		// std::cout << "End of Epoch:" << std::endl;
		return true;
	}
	return false;
}

bool cSimAdapter::agentHasFallen()
{
	// const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();

	// this->_scene->HasFallen();

	return this->_scene->HasFallen();
}

bool cSimAdapter::needUpdatedAction()
{
	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	std::shared_ptr<cTerrainRLCharController> controller =  std::static_pointer_cast< cTerrainRLCharController >(char_->GetController());

	int con_state_ = controller->GetState();
	// std::cout << "Controller State: " << con_state_ << " last controller state: "<< _lastControllerState << std::endl;
	if (con_state_ == 0 && (_lastControllerState != 0))
	{
		_lastControllerState = con_state_;
		/// This will update the sim to properly calculate the terrain input
		// std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioSimChar>(new cDrawScenarioSimChar(gCamera));
		// this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__->GetScene());
		return true;
	}
	_lastControllerState = con_state_;
	return false; // 0 = start of new action
}

void cSimAdapter::onKeyEvent(int key, int mouseX, int mouseY)
{
	// Globals::app->onKeyEvent(key, mouseX, mouseY);
	std::cout << "Trying key: " << (char(key)) << std::endl;
	std::shared_ptr<cDrawScenario> scenario__ = std::static_pointer_cast<cDrawScenario>(this->_gScenario);
				// std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioSimChar>(new cDrawScenarioSimChar(gCamera));
	// this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__->GetScene());
	// this->_gScenario = scenario__;
	if (scenario__ != NULL)
	{
		scenario__->Keyboard(key, mouseX, mouseY);
	}
}

void cSimAdapter::setRelativeFilePath(std::string relativePath)
{
	this->_relativePath = relativePath;
}

void cSimAdapter::setRandomSeed(int seed)
{
	this->_scene->SetRandSeed(seed);
}

size_t cSimAdapter::getActionSpaceSize() const
{
	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	std::shared_ptr<cTerrainRLCharController> controller =  std::static_pointer_cast< cTerrainRLCharController >(char_->GetController());
	return controller->GetPoliActionSize();
}

size_t cSimAdapter::getObservationSpaceSize() const
{

	return this->getState().size();
}


void cSimAdapter::handleUpdatedAction()
{
	auto sc = std::dynamic_pointer_cast<cScenarioExp>(this->_scene);
	if ( sc != nullptr )
	{
		sc->HandleNewActionUpdate();
	}
}

