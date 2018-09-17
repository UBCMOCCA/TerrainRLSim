#include "sim/TerrainRLCtrlFactory.h"

#include "sim/CtPDPhaseController.h"

const std::string gCharCtrlName[cTerrainRLCtrlFactory::eCharCtrlMax] = {
    "none", "dog", "dog_cacla", "dog_mace", "dog_dmace", "dog_dpg", "dog_mace_dpg", "goat_mace", "monoped_hopper",
    "monoped_hopper_cacla", "monoped_hopper_mace", "monoped_hopper_dmace", "raptor", "raptor_cacla", "raptor_mace",
    "raptor_dmace", "ct", "ct_rnn", "ct_pd", "ct_vel", "ct_npd", "ct_mtu", "ct_track", "ct_pd_track", "ct_vel_track",
    "ct_npd_track", "ct_mtu_track", "ct_phase", "ct_pd_phase", "ct_vel_phase", "ct_q", "ct_q_pd", "ct_q_vel",
    "ct_q_track", "ct_q_pd_track", "ct_q_vel_track", "ct_target", "ct_pd_track_target", "ct_pd_phase_target",
    "dog_cacla_dq", "biped", "biped2D", "biped2D_cacla", "biped2D_parameterized",
    // "biped3D",
    "biped3D_cacla", "waypoint", "waypoint_vel", "soccer", "biped3d_step", "biped2D_cacla_fd", "ct_target_terr",
    "ct_target_soccer"};

cTerrainRLCtrlFactory::tCtrlParams::tCtrlParams() {
    mCharCtrl = eCharCtrlNone;
    mCtrlParamFile = "";
    mCtrlParamFile = "";

    mChar = nullptr;
    mGround = nullptr;
    mGravity = tVector(0, -9.8, 0, 0);

    mCycleDur = 1;
    mCtQueryRate = 60; // policy queries per second

    // hack: don't override unless specified
    mNumGroundSamples = -1;
    mGroundSampleRes3d = -1;
    mViewDist = -1;

    mEnableSymmetricStep = false;
    mEnableSymmetricLLC = false;
    mWaypointInitStepLen = 0;

    InitNetFileArray(mNetFiles);
}

void cTerrainRLCtrlFactory::InitNetFileArray(std::vector<std::string> &out_arr) {
    out_arr.resize(eNetFileMax);
    for (int f = 0; f < eNetFileMax; ++f) {
        out_arr[f] = "";
    }
}

void cTerrainRLCtrlFactory::ParseCharCtrl(const std::string &char_ctrl_str, eCharCtrl &out_char_ctrl) {
    bool found = false;
    if (char_ctrl_str == "") {
        out_char_ctrl = eCharCtrlNone;
        found = true;
    } else {
        for (int i = 0; i < eCharCtrlMax; ++i) {
            const std::string &name = gCharCtrlName[i];
            if (char_ctrl_str == name) {
                out_char_ctrl = static_cast<eCharCtrl>(i);
                found = true;
                break;
            }
        }
    }

    if (!found) {
        assert(false && "Unsupported character controller"); // unsupported character controller
    }
}

bool cTerrainRLCtrlFactory::BuildController(const tCtrlParams &params, std::shared_ptr<cCharController> &out_ctrl) {
    bool succ = true;

    switch (params.mCharCtrl) {
    case eCharCtrlCtPDPhase:
        succ = BuildCtPDPhaseController(params, out_ctrl);
        break;
    default:
        assert(false && "Failed Building Unsupported Controller"); // unsupported controller
        break;
    }

    return succ;
}

bool cTerrainRLCtrlFactory::BuildDogController(const tCtrlParams &params, std::shared_ptr<cCharController> &out_ctrl) {
    bool succ = true;

    std::shared_ptr<cDogControllerQ> ctrl = std::shared_ptr<cDogControllerQ>(new cDogControllerQ());
    ctrl->SetGround(params.mGround);
    ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

    const std::string &poli_net_file = params.mNetFiles[eNetFileActor];
    const std::string &poli_model_file = params.mNetFiles[eNetFileActorModel];
    if (poli_net_file != "") {
        succ &= ctrl->LoadNet(poli_net_file);

        if (succ && poli_model_file != "") {
            ctrl->LoadModel(poli_model_file);
        }
    }

    out_ctrl = ctrl;

    return succ;
}

bool cTerrainRLCtrlFactory::BuildDogControllerCacla(const tCtrlParams &params,
                                                    std::shared_ptr<cCharController> &out_ctrl) {
    bool succ = true;

    std::shared_ptr<cDogControllerCacla> ctrl = std::shared_ptr<cDogControllerCacla>(new cDogControllerCacla());
    ctrl->SetGround(params.mGround);
    ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

    const std::string &poli_net_file = params.mNetFiles[eNetFileActor];
    const std::string &poli_model_file = params.mNetFiles[eNetFileActorModel];
    const std::string &critic_net_file = params.mNetFiles[eNetFileCritic];
    const std::string &critic_model_file = params.mNetFiles[eNetFileCriticModel];

    if (poli_net_file != "") {
        succ &= ctrl->LoadNet(poli_net_file);

        if (succ && poli_model_file != "") {
            ctrl->LoadModel(poli_model_file);
        }
    }

    if (critic_net_file != "") {
        bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
        succ &= critic_succ;
        if (critic_succ && critic_model_file != "") {
            ctrl->LoadCriticModel(critic_model_file);
        }
    }

    out_ctrl = ctrl;

    return succ;
}

bool cTerrainRLCtrlFactory::BuildDogControllerCaclaDQ(const tCtrlParams &params,
                                                      std::shared_ptr<cCharController> &out_ctrl) {
    bool succ = true;

    std::shared_ptr<cDogControllerCaclaDQ> ctrl = std::shared_ptr<cDogControllerCaclaDQ>(new cDogControllerCaclaDQ());
    ctrl->SetGround(params.mGround);
    ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

    const std::string &poli_net_file = params.mNetFiles[eNetFileActor];
    const std::string &poli_model_file = params.mNetFiles[eNetFileActorModel];
    const std::string &critic_net_file = params.mNetFiles[eNetFileCritic];
    const std::string &critic_model_file = params.mNetFiles[eNetFileCriticModel];

    if (poli_net_file != "") {
        succ &= ctrl->LoadNet(poli_net_file);

        if (succ && poli_model_file != "") {
            ctrl->LoadModel(poli_model_file);
        }
    }

    if (critic_net_file != "") {
        bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
        succ &= critic_succ;
        if (critic_succ && critic_model_file != "") {
            ctrl->LoadCriticModel(critic_model_file);
        }
    }

    out_ctrl = ctrl;

    return succ;
}

bool cTerrainRLCtrlFactory::BuildDogControllerMACE(const tCtrlParams &params,
                                                   std::shared_ptr<cCharController> &out_ctrl) {
    bool succ = true;

    std::shared_ptr<cDogControllerMACE> ctrl = std::shared_ptr<cDogControllerMACE>(new cDogControllerMACE());
    ctrl->SetGround(params.mGround);
    ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

    const std::string &poli_net_file = params.mNetFiles[eNetFileActor];
    const std::string &poli_model_file = params.mNetFiles[eNetFileActorModel];

    if (poli_net_file != "") {
        succ &= ctrl->LoadNet(poli_net_file);

        if (succ && poli_model_file != "") {
            ctrl->LoadModel(poli_model_file);
        }
    }

    out_ctrl = ctrl;

    return succ;
}

bool cTerrainRLCtrlFactory::BuildDogControllerDMACE(const tCtrlParams &params,
                                                    std::shared_ptr<cCharController> &out_ctrl) {
    bool succ = true;

    std::shared_ptr<cDogControllerDMACE> ctrl = std::shared_ptr<cDogControllerDMACE>(new cDogControllerDMACE());
    ctrl->SetGround(params.mGround);
    ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

    const std::string &poli_net_file = params.mNetFiles[eNetFileActor];
    const std::string &poli_model_file = params.mNetFiles[eNetFileActorModel];
    const std::string &critic_net_file = params.mNetFiles[eNetFileCritic];
    const std::string &critic_model_file = params.mNetFiles[eNetFileCriticModel];

    if (poli_net_file != "") {
        succ &= ctrl->LoadNet(poli_net_file);

        if (succ && poli_model_file != "") {
            ctrl->LoadModel(poli_model_file);
        }
    }

    if (critic_net_file != "") {
        bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
        succ &= critic_succ;
        if (critic_succ && critic_model_file != "") {
            ctrl->LoadCriticModel(critic_model_file);
        }
    }

    out_ctrl = ctrl;

    return succ;
}

bool cTerrainRLCtrlFactory::BuildDogControllerDPG(const tCtrlParams &params,
                                                  std::shared_ptr<cCharController> &out_ctrl) {
    bool succ = true;

    std::shared_ptr<cDogControllerDPG> ctrl = std::shared_ptr<cDogControllerDPG>(new cDogControllerDPG());
    ctrl->SetGround(params.mGround);
    ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

    const std::string &poli_net_file = params.mNetFiles[eNetFileActor];
    const std::string &poli_model_file = params.mNetFiles[eNetFileActorModel];
    const std::string &critic_net_file = params.mNetFiles[eNetFileCritic];
    const std::string &critic_model_file = params.mNetFiles[eNetFileCriticModel];

    if (poli_net_file != "") {
        succ &= ctrl->LoadNet(poli_net_file);

        if (succ && poli_model_file != "") {
            ctrl->LoadModel(poli_model_file);
        }
    }

    if (critic_net_file != "") {
        bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
        succ &= critic_succ;
        if (critic_succ && critic_model_file != "") {
            ctrl->LoadCriticModel(critic_model_file);
        }
    }

    out_ctrl = ctrl;

    return succ;
}

bool cTerrainRLCtrlFactory::BuildDogControllerMACEDPG(const tCtrlParams &params,
                                                      std::shared_ptr<cCharController> &out_ctrl) {
    bool succ = true;

    std::shared_ptr<cDogControllerMACEDPG> ctrl = std::shared_ptr<cDogControllerMACEDPG>(new cDogControllerMACEDPG());
    ctrl->SetGround(params.mGround);
    ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

    const std::string &poli_net_file = params.mNetFiles[eNetFileActor];
    const std::string &poli_model_file = params.mNetFiles[eNetFileActorModel];
    const std::string &critic_net_file = params.mNetFiles[eNetFileCritic];
    const std::string &critic_model_file = params.mNetFiles[eNetFileCriticModel];

    if (poli_net_file != "") {
        succ &= ctrl->LoadNet(poli_net_file);

        if (succ && poli_model_file != "") {
            ctrl->LoadModel(poli_model_file);
        }
    }

    if (critic_net_file != "") {
        bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
        succ &= critic_succ;
        if (critic_succ && critic_model_file != "") {
            ctrl->LoadCriticModel(critic_model_file);
        }
    }

    out_ctrl = ctrl;

    return succ;
}

bool cTerrainRLCtrlFactory::BuildGoatControllerMACE(const tCtrlParams &params,
                                                    std::shared_ptr<cCharController> &out_ctrl) {
    bool succ = true;

    std::shared_ptr<cGoatControllerMACE> ctrl = std::shared_ptr<cGoatControllerMACE>(new cGoatControllerMACE());
    ctrl->SetGround(params.mGround);
    ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

    const std::string &poli_net_file = params.mNetFiles[eNetFileActor];
    const std::string &poli_model_file = params.mNetFiles[eNetFileActorModel];

    if (poli_net_file != "") {
        succ &= ctrl->LoadNet(poli_net_file);

        if (succ && poli_model_file != "") {
            ctrl->LoadModel(poli_model_file);
        }
    }

    out_ctrl = ctrl;

    return succ;
}

bool cTerrainRLCtrlFactory::BuildMonopedHopperController(const tCtrlParams &params,
                                                         std::shared_ptr<cCharController> &out_ctrl) {
    bool succ = true;

    std::shared_ptr<cMonopedHopperControllerQ> ctrl =
        std::shared_ptr<cMonopedHopperControllerQ>(new cMonopedHopperControllerQ());
    ctrl->SetGround(params.mGround);
    ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

    const std::string &poli_net_file = params.mNetFiles[eNetFileActor];
    const std::string &poli_model_file = params.mNetFiles[eNetFileActorModel];

    if (poli_net_file != "") {
        succ &= ctrl->LoadNet(poli_net_file);

        if (succ && poli_model_file != "") {
            ctrl->LoadModel(poli_model_file);
        }
    }

    out_ctrl = ctrl;

    return succ;
}

bool cTerrainRLCtrlFactory::BuildMonopedHopperControllerCacla(const tCtrlParams &params,
                                                              std::shared_ptr<cCharController> &out_ctrl) {
    bool succ = true;

    std::shared_ptr<cMonopedHopperControllerCacla> ctrl =
        std::shared_ptr<cMonopedHopperControllerCacla>(new cMonopedHopperControllerCacla());
    ctrl->SetGround(params.mGround);
    ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

    const std::string &poli_net_file = params.mNetFiles[eNetFileActor];
    const std::string &poli_model_file = params.mNetFiles[eNetFileActorModel];
    const std::string &critic_net_file = params.mNetFiles[eNetFileCritic];
    const std::string &critic_model_file = params.mNetFiles[eNetFileCriticModel];

    if (poli_net_file != "") {
        succ &= ctrl->LoadNet(poli_net_file);

        if (succ && poli_model_file != "") {
            ctrl->LoadModel(poli_model_file);
        }
    }

    if (critic_net_file != "") {
        bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
        succ &= critic_succ;
        if (critic_succ && critic_model_file != "") {
            ctrl->LoadCriticModel(critic_model_file);
        }
    }

    out_ctrl = ctrl;

    return succ;
}

bool cTerrainRLCtrlFactory::BuildMonopedHopperControllerMACE(const tCtrlParams &params,
                                                             std::shared_ptr<cCharController> &out_ctrl) {
    bool succ = true;

    std::shared_ptr<cMonopedHopperControllerMACE> ctrl =
        std::shared_ptr<cMonopedHopperControllerMACE>(new cMonopedHopperControllerMACE());
    ctrl->SetGround(params.mGround);
    ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

    const std::string &poli_net_file = params.mNetFiles[eNetFileActor];
    const std::string &poli_model_file = params.mNetFiles[eNetFileActorModel];

    if (poli_net_file != "") {
        succ &= ctrl->LoadNet(poli_net_file);

        if (succ && poli_model_file != "") {
            ctrl->LoadModel(poli_model_file);
        }
    }

    out_ctrl = ctrl;

    return succ;
}

bool cTerrainRLCtrlFactory::BuildRaptorController(const tCtrlParams &params,
                                                  std::shared_ptr<cCharController> &out_ctrl) {
    bool succ = true;

    std::shared_ptr<cRaptorControllerQ> ctrl = std::shared_ptr<cRaptorControllerQ>(new cRaptorControllerQ());
    ctrl->SetGround(params.mGround);
    ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

    const std::string &poli_net_file = params.mNetFiles[eNetFileActor];
    const std::string &poli_model_file = params.mNetFiles[eNetFileActorModel];

    if (poli_net_file != "") {
        succ &= ctrl->LoadNet(poli_net_file);

        if (succ && poli_model_file != "") {
            ctrl->LoadModel(poli_model_file);
        }
    }

    out_ctrl = ctrl;

    return succ;
}

bool cTerrainRLCtrlFactory::BuildRaptorControllerCacla(const tCtrlParams &params,
                                                       std::shared_ptr<cCharController> &out_ctrl) {
    bool succ = true;

    std::shared_ptr<cRaptorControllerCacla> ctrl =
        std::shared_ptr<cRaptorControllerCacla>(new cRaptorControllerCacla());
    ctrl->SetGround(params.mGround);
    ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

    const std::string &poli_net_file = params.mNetFiles[eNetFileActor];
    const std::string &poli_model_file = params.mNetFiles[eNetFileActorModel];
    const std::string &critic_net_file = params.mNetFiles[eNetFileCritic];
    const std::string &critic_model_file = params.mNetFiles[eNetFileCriticModel];

    if (poli_net_file != "") {
        succ &= ctrl->LoadNet(poli_net_file);

        if (succ && poli_model_file != "") {
            ctrl->LoadModel(poli_model_file);
        }
    }

    if (critic_net_file != "") {
        bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
        succ &= critic_succ;
        if (critic_succ && critic_model_file != "") {
            ctrl->LoadCriticModel(critic_model_file);
        }
    }

    out_ctrl = ctrl;

    return succ;
}

bool cTerrainRLCtrlFactory::BuildRaptorControllerMACE(const tCtrlParams &params,
                                                      std::shared_ptr<cCharController> &out_ctrl) {
    bool succ = true;

    std::shared_ptr<cRaptorControllerMACE> ctrl = std::shared_ptr<cRaptorControllerMACE>(new cRaptorControllerMACE());
    ctrl->SetGround(params.mGround);
    ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

    const std::string &poli_net_file = params.mNetFiles[eNetFileActor];
    const std::string &poli_model_file = params.mNetFiles[eNetFileActorModel];

    if (poli_net_file != "") {
        succ &= ctrl->LoadNet(poli_net_file);

        if (succ && poli_model_file != "") {
            ctrl->LoadModel(poli_model_file);
        }
    }

    out_ctrl = ctrl;

    return succ;
}

bool cTerrainRLCtrlFactory::BuildBipedController(const tCtrlParams &params,
                                                 std::shared_ptr<cCharController> &out_ctrl) {
    bool succ = true;

    std::shared_ptr<cBipedControllerQ> ctrl = std::shared_ptr<cBipedControllerQ>(new cBipedControllerQ());
    ctrl->SetGround(params.mGround);
    ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

    const std::string &poli_net_file = params.mNetFiles[eNetFileActor];
    const std::string &poli_model_file = params.mNetFiles[eNetFileActorModel];

    if (poli_net_file != "") {
        succ &= ctrl->LoadNet(poli_net_file);

        if (succ && poli_model_file != "") {
            ctrl->LoadModel(poli_model_file);
        }
    }

    out_ctrl = ctrl;

    return succ;
}

bool cTerrainRLCtrlFactory::BuildBipedController2D(const tCtrlParams &params,
                                                   std::shared_ptr<cCharController> &out_ctrl) {
    bool succ = true;

    std::shared_ptr<cBipedController2DQ> ctrl = std::shared_ptr<cBipedController2DQ>(new cBipedController2DQ());
    ctrl->SetGround(params.mGround);
    ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

    const std::string &poli_net_file = params.mNetFiles[eNetFileActor];
    const std::string &poli_model_file = params.mNetFiles[eNetFileActorModel];

    if (poli_net_file != "") {
        succ &= ctrl->LoadNet(poli_net_file);

        if (succ && poli_model_file != "") {
            ctrl->LoadModel(poli_model_file);
        }
    }

    out_ctrl = ctrl;

    return succ;
}

bool cTerrainRLCtrlFactory::BuildBipedController2DCACLA(const tCtrlParams &params,
                                                        std::shared_ptr<cCharController> &out_ctrl) {
    bool succ = true;

    std::shared_ptr<cBipedController2DCACLA> ctrl =
        std::shared_ptr<cBipedController2DCACLA>(new cBipedController2DCACLA());
    // ctrl->setRelativeFilePath(params.mRelativeFilePath);
    ctrl->SetGround(params.mGround);
    ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);
    if (params.mNumGroundSamples >= 0) {
        ctrl->SetNumGroundSamples(params.mNumGroundSamples);
    }
    if (params.mViewDist >= 0) {
        ctrl->SetViewDist(params.mViewDist);
    }
    const std::string &poli_net_file = params.mNetFiles[eNetFileActor];
    const std::string &poli_model_file = params.mNetFiles[eNetFileActorModel];
    const std::string &critic_net_file = params.mNetFiles[eNetFileCritic];
    const std::string &critic_model_file = params.mNetFiles[eNetFileCriticModel];

    if (poli_net_file != "") {
        succ &= ctrl->LoadNet(poli_net_file);

        if (succ && poli_model_file != "") {
            ctrl->LoadModel(poli_model_file);
        }
    }

    if (critic_net_file != "") {
        bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
        succ &= critic_succ;
        if (critic_succ && critic_model_file != "") {
            ctrl->LoadCriticModel(critic_model_file);
        }
    }

    out_ctrl = ctrl;

    return succ;
}

bool cTerrainRLCtrlFactory::BuildBipedController2DCaclaFD(const tCtrlParams &params,
                                                          std::shared_ptr<cCharController> &out_ctrl) {
    bool succ = true;

    std::shared_ptr<cBipedController2DCaclaFD> ctrl =
        std::shared_ptr<cBipedController2DCaclaFD>(new cBipedController2DCaclaFD());
    ctrl->SetGround(params.mGround);
    ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

    const std::string &poli_net_file = params.mNetFiles[eNetFileActor];
    const std::string &poli_model_file = params.mNetFiles[eNetFileActorModel];
    const std::string &critic_net_file = params.mNetFiles[eNetFileCritic];
    const std::string &critic_model_file = params.mNetFiles[eNetFileCriticModel];
    const std::string &fd_net_file = params.mNetFiles[eNetFileForwardDynamics];
    const std::string &fd_model_file = params.mNetFiles[eNetFileForwardDynamicsModel];

    if (poli_net_file != "") {
        succ &= ctrl->LoadNet(poli_net_file);

        if (succ && poli_model_file != "") {
            ctrl->LoadModel(poli_model_file);
        }
    }

    if (critic_net_file != "") {
        bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
        succ &= critic_succ;
        if (critic_succ && critic_model_file != "") {
            ctrl->LoadCriticModel(critic_model_file);
        }
    }

    if (fd_net_file != "") {

        bool fd_succ = ctrl->LoadForwardDynamicsNet(fd_net_file);
        succ &= fd_succ;
        if (fd_succ && (fd_model_file != "")) {
            ctrl->LoadForwardDynamicsModel(fd_model_file);
        }
    }

    out_ctrl = ctrl;

    return succ;
}

bool cTerrainRLCtrlFactory::BuildBipedController2DParameterized(const tCtrlParams &params,
                                                                std::shared_ptr<cCharController> &out_ctrl) {
    bool succ = true;

    std::shared_ptr<cBipedController2DParameterized> ctrl =
        std::shared_ptr<cBipedController2DParameterized>(new cBipedController2DParameterized());
    ctrl->SetGround(params.mGround);
    ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

    const std::string &poli_net_file = params.mNetFiles[eNetFileActor];
    const std::string &poli_model_file = params.mNetFiles[eNetFileActorModel];
    const std::string &critic_net_file = params.mNetFiles[eNetFileCritic];
    const std::string &critic_model_file = params.mNetFiles[eNetFileCriticModel];

    if (poli_net_file != "") {
        succ &= ctrl->LoadNet(poli_net_file);

        if (succ && poli_model_file != "") {
            ctrl->LoadModel(poli_model_file);
        }
    }

    if (critic_net_file != "") {
        bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
        succ &= critic_succ;
        if (critic_succ && critic_model_file != "") {
            ctrl->LoadCriticModel(critic_model_file);
        }
    }

    out_ctrl = ctrl;

    return succ;
}

bool cTerrainRLCtrlFactory::BuildCtPDPhaseController(const tCtrlParams &params,
                                                     std::shared_ptr<cCharController> &out_ctrl) {
    const double update_period = 1 / params.mCtQueryRate;
    bool succ = true;

    std::shared_ptr<cCtPDPhaseController> ctrl = std::shared_ptr<cCtPDPhaseController>(new cCtPDPhaseController());
    ctrl->SetGround(params.mGround);
    ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);
    ctrl->SetUpdatePeriod(update_period);
    ctrl->SetCycleDur(params.mCycleDur);
    if (params.mNumGroundSamples >= 0) {
        ctrl->SetNumGroundSamples(params.mNumGroundSamples);
    }
    if (params.mViewDist >= 0) {
        ctrl->SetViewDist(params.mViewDist);
    }
    const std::string &poli_net_file = params.mNetFiles[eNetFileActor];
    const std::string &poli_model_file = params.mNetFiles[eNetFileActorModel];
    const std::string &critic_net_file = params.mNetFiles[eNetFileCritic];
    const std::string &critic_model_file = params.mNetFiles[eNetFileCriticModel];

    if (poli_net_file != "") {
        succ &= ctrl->LoadNet(poli_net_file);
        if (succ && poli_model_file != "") {
            ctrl->LoadModel(poli_model_file);
        }
    }

    if (critic_net_file != "") {
        bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
        succ &= critic_succ;
        if (critic_succ && critic_model_file != "") {
            ctrl->LoadCriticModel(critic_model_file);
        }
    }

    out_ctrl = ctrl;

    return succ;
}
