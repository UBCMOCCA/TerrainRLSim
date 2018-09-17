#pragma once

#include <memory>
#include <string>
#include <vector>

#include "sim/Ground.h"
#include "sim/SimCharacter.h"
#include "sim/TerrainRLCharController.h"

class cTerrainRLCtrlFactory {
  public:
    enum eCharCtrl {
        eCharCtrlNone,
        eCharCtrlDog,
        eCharCtrlDogCacla,
        eCharCtrlDogMACE,
        eCharCtrlDogDMACE,
        eCharCtrlDogDPG,
        eCharCtrlDogMACEDPG,
        eCharCtrlGoatMACE,
        eCharCtrlMonopedHopper,
        eCharCtrlMonopedHopperCacla,
        eCharCtrlMonopedHopperMACE,
        eCharCtrlMonopedHopperDMACE,
        eCharCtrlRaptor,
        eCharCtrlRaptorCacla,
        eCharCtrlRaptorMACE,
        eCharCtrlRaptorDMACE,
        eCharCtrlCt,
        eCharCtrlCtRNN,
        eCharCtrlCtPD,
        eCharCtrlCtVel,
        eCharCtrlCtNPD,
        eCharCtrlCtMTU,
        eCharCtrlCtTrack,
        eCharCtrlCtPDTrack,
        eCharCtrlCtVelTrack,
        eCharCtrlCtNPDTrack,
        eCharCtrlCtMTUTrack,
        eCharCtrlCtPhase,
        eCharCtrlCtPDPhase,
        eCharCtrlCtVelPhase,
        eCharCtrlCtQ,
        eCharCtrlCtQPD,
        eCharCtrlCtQVel,
        eCharCtrlCtQTrack,
        eCharCtrlCtQPDTrack,
        eCharCtrlCtQVelTrack,
        eCharCtrlCtTarget,
        eCharCtrlCtPDTrackTarget,
        eCharCtrlCtPDPhaseTarget,
        eCharCtrlDogCaclaDQ,
        eCharCtrlBiped,
        eCharCtrlBiped2D,
        eCharCtrlBiped2DCACLA,
        eCharCtrlBiped2DParameterized,
        // eCharCtrlBiped3D,
        eCharCtrlBiped3DCacla,
        eCharCtrlWaypoint,
        eCharCtrlWaypointVel,
        eCharCtrlSoccer,
        eCharCtrlBiped3DStep,
        eCharCtrlBiped2DCaclaFD,
        eCharCtrlCtTargetTerr,
        eCharCtrlCtTargetSoccer,
        eCharCtrlMax
    };

    struct tCtrlParams {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        eCharCtrl mCharCtrl;
        std::vector<std::string> mNetFiles;
        std::string mCtrlParamFile;

        std::shared_ptr<cSimCharacter> mChar;
        std::shared_ptr<cGround> mGround;
        tVector mGravity;

        double mCycleDur;
        double mCtQueryRate;

        // 2D terrain samples
        int mNumGroundSamples;
        // 3D terrain samples
        int mGroundSampleRes3d;
        //
        int mViewDist;

        // waypoint params
        bool mEnableSymmetricStep;
        bool mEnableSymmetricLLC;
        double mWaypointInitStepLen;

        tCtrlParams();
    };

    enum eNetFile {
        eNetFileActor,
        eNetFileCritic,
        eNetFileActor1,
        eNetFileCritic1,

        eNetFileActorSolver,
        eNetFileCriticSolver,
        eNetFileActor1Solver,
        eNetFileCritic1Solver,

        eNetFileActorModel,
        eNetFileCriticModel,
        eNetFileActor1Model,
        eNetFileCritic1Model,
        eNetFileActorTerrainModel,
        eNetFileActorActionModel,
        eNetFileCriticTerrainModel,
        eNetFileCriticValueModel,
        eNetFileForwardDynamics,
        eNetFileForwardDynamicsSolver,
        eNetFileForwardDynamicsModel,
        eNetFileMax
    };

    static void InitNetFileArray(std::vector<std::string> &out_arr);
    static void ParseCharCtrl(const std::string &char_ctrl_str, eCharCtrl &out_char_ctrl);
    static bool BuildController(const tCtrlParams &params, std::shared_ptr<cCharController> &out_ctrl);

  protected:
    static bool BuildCtPDPhaseController(const tCtrlParams &params, std::shared_ptr<cCharController> &out_ctrl);
};
