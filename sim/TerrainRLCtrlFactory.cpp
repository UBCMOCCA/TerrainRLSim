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

    out_ctrl = ctrl;

    return succ;
}
