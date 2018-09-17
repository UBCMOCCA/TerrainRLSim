/*
 * SimAdapter.cpp
 *
 *  Created on: 2017-01-20
 *      Author: gberseth
 */

#include "SimAdapter.h"
#include "Main.h"
#include "scenarios/DrawScenarioImitateEval.h"
#include "scenarios/ScenarioExp.h"
#include "scenarios/ScenarioImitateEval.h"
#include "sim/TerrainRLCharController.h"

cSimAdapter::cSimAdapter(std::vector<std::string> args) {
    // TODO Auto-generated constructor stub
    _lastControllerState = 0;
    _render = false;
    _gScenario = nullptr;
    _args = args;
    this->_relativePath = "";
}

cSimAdapter::~cSimAdapter() {
    // TODO Auto-generated destructor stub
}

void cSimAdapter::setRender(bool shouldRender) { this->_render = shouldRender; }

void cSimAdapter::init() {
    char **cstrings = new char *[_args.size()];
    for (size_t i = 0; i < _args.size(); ++i) {
        cstrings[i] = new char[_args[i].size() + 1];
        std::strcpy(cstrings[i], _args[i].c_str());
    }

    int argc = _args.size();

    gArgc = argc;
    gArgv = cstrings;
    ParseArgs(gArgc, gArgv);

    std::string scenario_name = "";
    gArgParser->ParseString("scenario", scenario_name);

    if (scenario_name == "hike_eval") {
        gCameraPosition = tVector(0, 50, 100, 0);
    }

    if (_render) {
        glutInit(&gArgc, gArgv);
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
        glutInitWindowSize(gWinWidth, gWinHeight);
        glutCreateWindow("Terrain RL");

        InitOpenGl();
    }

    if (_render) {
        InitCamera();
        ClearScenario();

        if (scenario_name == "imitate_eval") {
            std::shared_ptr<cDrawScenarioSimChar> scenario__ =
                std::shared_ptr<cDrawScenarioImitateEval>(new cDrawScenarioImitateEval(gCamera));
            this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__->GetScene());
            this->_gScenario = scenario__;
            if (this->_gScenario != NULL) {
                auto sim_char_scene = std::dynamic_pointer_cast<cDrawScenarioTerrainRL>(scenario__);
                if (sim_char_scene != nullptr) {
                    sim_char_scene->SetOutputTex(gIntermediateFrameBuffer);
                }
            }
            gScenario = std::shared_ptr<cDrawScenario>(scenario__);
            this->_gScenario = gScenario;
        }  else {
            std::cerr << "scenario not recognized: " << scenario_name << std::endl;
            exit(-1);
        }
    }
    else {
        ClearScenario();
        if (scenario_name == "imitate_eval") {
            std::shared_ptr<cScenarioSimChar> scenario__ =
                std::shared_ptr<cScenarioSimChar>(new cScenarioImitateEval());
            this->_scene = std::dynamic_pointer_cast<cScenarioSimChar>(scenario__);
            this->_gScenario = scenario__;
        }  else {
            std::cout << "Scenario type not yet supported by adapter: " << scenario_name << std::endl;
        }
    }

    this->_gScenario->ParseArgs(gArgParser);
    this->_gScenario->Init();
    printf("Loaded scenario: %s\n", this->_gScenario->GetName().c_str());
    if (_render) {
        this->_scene = std::shared_ptr<cScenarioSimChar>(
            std::dynamic_pointer_cast<cDrawScenarioSimChar>(this->_gScenario)->GetScene());
        Reshape(gWinWidth, gWinHeight);
        glutDisplayFunc(Display);
        glutReshapeFunc(Reshape);
        glutKeyboardFunc(Keyboard);
        glutMouseFunc(MouseClick);
        glutMotionFunc(MouseMove);
    }

    InitTime();

    for (size_t i = 0; i < _args.size(); ++i) {
        // delete[] cstrings[i];
    }
}

void cSimAdapter::initEpoch() {

    this->_gScenario->Reset();
    gForceClear = true;
}

void cSimAdapter::update() {

    if (gAnimate) {
        if (_render) {
            int num_steps = GetNumTimeSteps();
            int current_time = glutGet(GLUT_ELAPSED_TIME);
            int elapsedTime = current_time - gPrevTime;

            gPrevTime = current_time;

            double timestep = (gPlaybackSpeed < 0) ? -gAnimStep : gAnimStep;

            if (this->_gScenario != NULL) {

                for (size_t i = 0; i < num_steps; ++i) {
                    this->_gScenario->Update(timestep);
                }
            }

            int update_dur = glutGet(GLUT_ELAPSED_TIME) - current_time;

            gUpdatesPerSec = (num_steps / (elapsedTime * 0.001));
            int timer_step = CalcDisplayAnimTime();
            timer_step -= update_dur;
            timer_step = std::max(timer_step, 0);

        } else {
            int num_steps = GetNumTimeSteps();

            double timestep = (gPlaybackSpeed < 0) ? -gAnimStep : gAnimStep;
            timestep = timestep;

            if (this->_gScenario != NULL) {

                for (size_t i = 0; i < num_steps; ++i) {
                    this->_gScenario->Update(timestep);
                }
            }
        }
    }

    if (this->_gScenario != nullptr) {
        if (this->_gScenario->IsDone()) {
            Shutdown();
        }
    }
}

void cSimAdapter::display() {
    if (_render) {
        Display();
    }
}

void cSimAdapter::finish() {
    if (this->_gScenario != nullptr) {
        if (this->_gScenario->IsDone()) {
            Shutdown();
        }
    }
}

double cSimAdapter::calcReward() {

    double reward_ = this->_scene->CalcReward();

    return reward_;
}

std::vector<double> cSimAdapter::calcRewards() {

    std::vector<double> reward_ = this->_scene->CalcRewards();

    return reward_;
}

double cSimAdapter::calcVelocity() {
    const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
    return char_->GetRootVel()(0);
}

bool cSimAdapter::hasStumbled() {
    return this->_scene->HasStumbled();
}

double cSimAdapter::jointTorque() {
    const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
    size_t num_joints = char_->GetNumJoints();
    double torque_sum = 0;
    for (size_t i = 0; i < num_joints; ++i) {

        cJoint &joint = char_->GetJoint(i);
        tVector torque(0, 0, 0, 0);
        if (joint.IsValid()) {
            torque = joint.GetTotalTorque();
        }
        /// Only for 2D for now...
        torque_sum += fabs(torque[2]);
    }
    /// divided by max torque (300)
    torque_sum = (torque_sum / (double)num_joints) / 300.0;
    return torque_sum;
}

double cSimAdapter::updateAction(std::vector<double> act) {
    const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
    std::shared_ptr<cTerrainRLCharController> controller =
        std::static_pointer_cast<cTerrainRLCharController>(char_->GetController());

    cTerrainRLCharController::tAction action;
    action.mID;
    action.mParams = Eigen::VectorXd::Zero(act.size());
    for (size_t i = 0; i < act.size(); i++) {
        action.mParams(i) = act[i];
    }

    controller->ApplyAction(action);
    return 0.0;
}

void cSimAdapter::act(std::vector<double> act) {

    const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
    std::shared_ptr<cTerrainRLCharController> controller =
        std::static_pointer_cast<cTerrainRLCharController>(char_->GetController());

    cTerrainRLCharController::tAction action;
    action.mID;
    action.mParams = Eigen::VectorXd::Zero(act.size());
    for (size_t i = 0; i < act.size(); i++) {
        action.mParams(i) = act[i];
    }

    controller->ApplyAction(action);
}

std::vector<double> cSimAdapter::getState() const {
    Eigen::VectorXd state;
    const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
    std::shared_ptr<cTerrainRLCharController> controller =
        std::static_pointer_cast<cTerrainRLCharController>(char_->GetController());
    controller->ParseGround();
    controller->BuildPoliState(state);

    std::vector<double> out(state.data(), state.data() + state.rows() * state.cols());
    return out;
}

bool cSimAdapter::endOfEpoch() {

    const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();

    if (char_->HasFallen() || char_->HasExploded()) {
        return true;
    }
    return false;
}

bool cSimAdapter::agentHasFallen() {
    return this->_scene->HasFallen();
}

bool cSimAdapter::needUpdatedAction() {
    const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
    std::shared_ptr<cTerrainRLCharController> controller =
        std::static_pointer_cast<cTerrainRLCharController>(char_->GetController());

    int con_state_ = controller->GetState();
    if (con_state_ == 0 && (_lastControllerState != 0)) {
        _lastControllerState = con_state_;
        return true;
    }
    _lastControllerState = con_state_;
    return false; // 0 = start of new action
}

void cSimAdapter::onKeyEvent(int key, int mouseX, int mouseY) {
    std::cout << "Trying key: " << (char(key)) << std::endl;
    std::shared_ptr<cDrawScenario> scenario__ = std::static_pointer_cast<cDrawScenario>(this->_gScenario);
    if (scenario__ != NULL) {
        scenario__->Keyboard(key, mouseX, mouseY);
    }
}

void cSimAdapter::setRelativeFilePath(std::string relativePath) { this->_relativePath = relativePath; }

void cSimAdapter::setRandomSeed(int seed) { this->_scene->SetRandSeed(seed); }

size_t cSimAdapter::getActionSpaceSize() const {
    const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
    std::shared_ptr<cTerrainRLCharController> controller =
        std::static_pointer_cast<cTerrainRLCharController>(char_->GetController());
    return controller->GetPoliActionSize();
}

size_t cSimAdapter::getObservationSpaceSize() const { return this->getState().size(); }

void cSimAdapter::handleUpdatedAction() {
    auto sc = std::dynamic_pointer_cast<cScenarioExp>(this->_scene);
    if (sc != nullptr) {
        sc->HandleNewActionUpdate();
    }
}
