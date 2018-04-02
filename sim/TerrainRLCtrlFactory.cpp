#include "sim/TerrainRLCtrlFactory.h"

#include "sim/DogControllerQ.h"
#include "sim/DogControllerCacla.h"
#include "sim/DogControllerCaclaDQ.h"
#include "sim/DogControllerMACE.h"
#include "sim/DogControllerDMACE.h"
#include "sim/DogControllerDPG.h"
#include "sim/DogControllerMACEDPG.h"
#include "sim/GoatControllerMACE.h"
#include "sim/MonopedHopperControllerQ.h"
#include "sim/MonopedHopperControllerCacla.h"
#include "sim/MonopedHopperControllerMACE.h"
#include "sim/RaptorControllerQ.h"
#include "sim/RaptorControllerCacla.h"
#include "sim/RaptorControllerMACE.h"
#include "sim/BipedControllerQ.h"
#include "sim/BipedController2DQ.h"
#include "sim/BipedController2DCACLA.h"
#include "sim/BipedController2DCaclaFD.h"
#include "sim/BipedController2DParameterized.h"
#include "sim/BipedController3D.h"
#include "sim/BipedController3DCACLA.h"
#include "sim/CtController.h"
#include "sim/CtRNNController.h"
#include "sim/CtPDController.h"
#include "sim/CtVelController.h"
#include "sim/CtNPDController.h"
#include "sim/CtMTUController.h"
#include "sim/CtTrackController.h"
#include "sim/CtPDTrackController.h"
#include "sim/CtVelTrackController.h"
#include "sim/CtNPDTrackController.h"
#include "sim/CtMTUTrackController.h"
#include "sim/CtPhaseController.h"
#include "sim/CtPDPhaseController.h"
#include "sim/CtVelPhaseController.h"
#include "sim/CtQTrackController.h"
#include "sim/CtQPDController.h"
#include "sim/CtQVelController.h"
#include "sim/CtQTrackController.h"
#include "sim/CtQPDTrackController.h"
#include "sim/CtQVelTrackController.h"
#include "sim/CtTargetController.h"
#include "sim/CtPDTrackTargetController.h"
#include "sim/CtPDPhaseTargetController.h"
#include "sim/WaypointController.h"
#include "sim/WaypointVelController.h"
#include "sim/SoccerController.h"
#include "sim/BipedStepController3D.h"
#include "sim/CtTargetTerrController.h"
#include "sim/CtTargetSoccerController.h"

const std::string gCharCtrlName[cTerrainRLCtrlFactory::eCharCtrlMax] =
{
	"none",
	"dog",
	"dog_cacla",
	"dog_mace",
	"dog_dmace",
	"dog_dpg",
	"dog_mace_dpg",
	"goat_mace",
	"monoped_hopper",
	"monoped_hopper_cacla",
	"monoped_hopper_mace",
	"monoped_hopper_dmace",
	"raptor",
	"raptor_cacla",
	"raptor_mace",
	"raptor_dmace",
	"ct",
	"ct_rnn",
	"ct_pd",
	"ct_vel",
	"ct_npd",
	"ct_mtu",
	"ct_track",
	"ct_pd_track",
	"ct_vel_track",
	"ct_npd_track",
	"ct_mtu_track",
	"ct_phase",
	"ct_pd_phase",
	"ct_vel_phase",
	"ct_q",
	"ct_q_pd",
	"ct_q_vel",
	"ct_q_track",
	"ct_q_pd_track",
	"ct_q_vel_track",
	"ct_target",
	"ct_pd_track_target",
	"ct_pd_phase_target",
	"dog_cacla_dq",
	"biped",
	"biped2D",
	"biped2D_cacla",
	"biped2D_parameterized",
	// "biped3D",
	"biped3D_cacla",
	"waypoint",
	"waypoint_vel",
	"soccer",
	"biped3d_step",
	"biped2D_cacla_fd",
	"ct_target_terr",
	"ct_target_soccer"
};

cTerrainRLCtrlFactory::tCtrlParams::tCtrlParams()
{
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

void cTerrainRLCtrlFactory::InitNetFileArray(std::vector<std::string>& out_arr)
{
	out_arr.resize(eNetFileMax);
	for (int f = 0; f < eNetFileMax; ++f)
	{
		out_arr[f] = "";
	}
}

void cTerrainRLCtrlFactory::ParseCharCtrl(const std::string& char_ctrl_str, eCharCtrl& out_char_ctrl)
{
	bool found = false;
	if (char_ctrl_str == "")
	{
		out_char_ctrl = eCharCtrlNone;
		found = true;
	}
	else
	{
		for (int i = 0; i < eCharCtrlMax; ++i)
		{
			const std::string& name = gCharCtrlName[i];
			if (char_ctrl_str == name)
			{
				out_char_ctrl = static_cast<eCharCtrl>(i);
				found = true;
				break;
			}
		}
	}

	if (!found)
	{
		assert(false && "Unsupported character controller"); // unsupported character controller
	}
}

bool cTerrainRLCtrlFactory::BuildController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	switch (params.mCharCtrl)
	{
	case eCharCtrlNone:
		std::cout << "Improper character controller type selected" << std::endl;
		break;
	case eCharCtrlDog:
		succ = BuildDogController(params, out_ctrl);
		break;
	case eCharCtrlDogCacla:
		succ = BuildDogControllerCacla(params, out_ctrl);
		break;
	case eCharCtrlDogMACE:
		succ = BuildDogControllerMACE(params, out_ctrl);
		break;
	case eCharCtrlDogDMACE:
		succ = BuildDogControllerDMACE(params, out_ctrl);
		break;
	case eCharCtrlDogDPG:
		succ = BuildDogControllerDPG(params, out_ctrl);
		break;
	case eCharCtrlDogMACEDPG:
		succ = BuildDogControllerMACEDPG(params, out_ctrl);
		break;
	case eCharCtrlGoatMACE:
		succ = BuildGoatControllerMACE(params, out_ctrl);
		break;
	case eCharCtrlMonopedHopper:
		succ = BuildMonopedHopperController(params, out_ctrl);
		break;
	case eCharCtrlMonopedHopperCacla:
		succ = BuildMonopedHopperControllerCacla(params, out_ctrl);
		break;
	case eCharCtrlMonopedHopperMACE:
	case eCharCtrlMonopedHopperDMACE:
		succ = BuildMonopedHopperControllerMACE(params, out_ctrl);
		break;
	case eCharCtrlRaptor:
		succ = BuildRaptorController(params, out_ctrl);
		break;
	case eCharCtrlRaptorCacla:
		succ = BuildRaptorControllerCacla(params, out_ctrl);
		break;
	case eCharCtrlRaptorMACE:
	case eCharCtrlRaptorDMACE:
		succ = BuildRaptorControllerMACE(params, out_ctrl);
		break;
	case eCharCtrlCt:
		succ = BuildCtController(params, out_ctrl);
		break;
	case eCharCtrlCtRNN:
		succ = BuildCtRNNController(params, out_ctrl);
		break;
	case eCharCtrlCtPD:
		succ = BuildCtPDController(params, out_ctrl);
		break;
	case eCharCtrlCtVel:
		succ = BuildCtVelController(params, out_ctrl);
		break;
	case eCharCtrlCtNPD:
		succ = BuildCtNPDController(params, out_ctrl);
		break;
	case eCharCtrlCtMTU:
		succ = BuildCtMTUController(params, out_ctrl);
		break;
	case eCharCtrlCtTrack:
		succ = BuildCtTrackController(params, out_ctrl);
		break;
	case eCharCtrlCtPDTrack:
		succ = BuildCtPDTrackController(params, out_ctrl);
		break;
	case eCharCtrlCtVelTrack:
		succ = BuildCtVelTrackController(params, out_ctrl);
		break;
	case eCharCtrlCtNPDTrack:
		succ = BuildCtNPDTrackController(params, out_ctrl);
		break;
	case eCharCtrlCtMTUTrack:
		succ = BuildCtMTUTrackController(params, out_ctrl);
		break;
	case eCharCtrlCtPhase:
		succ = BuildCtPhaseController(params, out_ctrl);
		break;
	case eCharCtrlCtPDPhase:
		succ = BuildCtPDPhaseController(params, out_ctrl);
		break;
	case eCharCtrlCtVelPhase:
		succ = BuildCtVelPhaseController(params, out_ctrl);
		break;
	case eCharCtrlCtQ:
		succ = BuildCtQController(params, out_ctrl);
		break;
	case eCharCtrlCtQPD:
		succ = BuildCtQPDController(params, out_ctrl);
		break;
	case eCharCtrlCtQVel:
		succ = BuildCtQVelController(params, out_ctrl);
		break;
	case eCharCtrlCtQTrack:
		succ = BuildCtQTrackController(params, out_ctrl);
		break;
	case eCharCtrlCtQPDTrack:
		succ = BuildCtQPDTrackController(params, out_ctrl);
		break;
	case eCharCtrlCtQVelTrack:
		succ = BuildCtQVelTrackController(params, out_ctrl);
		break;
	case eCharCtrlCtTarget:
		succ = BuildCtTargetController(params, out_ctrl);
		break;
	case eCharCtrlCtPDTrackTarget:
		succ = BuildCtPDTrackTargetController(params, out_ctrl);
		break;
	case eCharCtrlCtPDPhaseTarget:
		succ = BuildCtPDPhaseTargetController(params, out_ctrl);
		break;
	case eCharCtrlDogCaclaDQ:
		succ = BuildDogControllerCaclaDQ(params, out_ctrl);
		break;
	case eCharCtrlBiped:
		succ = BuildBipedController(params, out_ctrl);
		break;
	case eCharCtrlBiped2D:
		succ = BuildBipedController2D(params, out_ctrl);
		break;
	case eCharCtrlBiped2DCACLA:
		succ = BuildBipedController2DCACLA(params, out_ctrl);
		break;
	case eCharCtrlBiped2DParameterized:
		succ = BuildBipedController2DParameterized(params, out_ctrl);
		break;
	case eCharCtrlBiped3DCacla:
		succ = BuildBipedController3DCACLA(params, out_ctrl);
		break;
	case eCharCtrlWaypoint:
		succ = BuildWaypointController(params, out_ctrl);
		break;
	case eCharCtrlWaypointVel:
		succ = BuildWaypointVelController(params, out_ctrl);
		break;
	case eCharCtrlSoccer:
		succ = BuildSoccerController(params, out_ctrl);
		break;
	case eCharCtrlBiped3DStep:
		succ = BuildBipedStepController3D(params, out_ctrl);
		break;
	case eCharCtrlBiped2DCaclaFD:
		succ = BuildBipedController2DCaclaFD(params, out_ctrl);
		break;
	case eCharCtrlCtTargetTerr:
		succ = BuildCtTargetTerrController(params, out_ctrl);
		break;
	case eCharCtrlCtTargetSoccer:
		succ = BuildCtTargetSoccerController(params, out_ctrl);
		break;
	default:
		assert(false && "Failed Building Unsupported Controller"); // unsupported controller
		break;
	}

	return succ;
}

bool cTerrainRLCtrlFactory::BuildDogController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cDogControllerQ> ctrl = std::shared_ptr<cDogControllerQ>(new cDogControllerQ());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildDogControllerCacla(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cDogControllerCacla> ctrl = std::shared_ptr<cDogControllerCacla>(new cDogControllerCacla());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildDogControllerCaclaDQ(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cDogControllerCaclaDQ> ctrl = std::shared_ptr<cDogControllerCaclaDQ>(new cDogControllerCaclaDQ());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildDogControllerMACE(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cDogControllerMACE> ctrl = std::shared_ptr<cDogControllerMACE>(new cDogControllerMACE());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	
	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildDogControllerDMACE(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cDogControllerDMACE> ctrl = std::shared_ptr<cDogControllerDMACE>(new cDogControllerDMACE());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildDogControllerDPG(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cDogControllerDPG> ctrl = std::shared_ptr<cDogControllerDPG>(new cDogControllerDPG());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildDogControllerMACEDPG(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cDogControllerMACEDPG> ctrl = std::shared_ptr<cDogControllerMACEDPG>(new cDogControllerMACEDPG());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildGoatControllerMACE(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cGoatControllerMACE> ctrl = std::shared_ptr<cGoatControllerMACE>(new cGoatControllerMACE());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	
	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildMonopedHopperController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cMonopedHopperControllerQ> ctrl = std::shared_ptr<cMonopedHopperControllerQ>(new cMonopedHopperControllerQ());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	
	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildMonopedHopperControllerCacla(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cMonopedHopperControllerCacla> ctrl = std::shared_ptr<cMonopedHopperControllerCacla>(new cMonopedHopperControllerCacla());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildMonopedHopperControllerMACE(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cMonopedHopperControllerMACE> ctrl = std::shared_ptr<cMonopedHopperControllerMACE>(new cMonopedHopperControllerMACE());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	
	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildRaptorController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cRaptorControllerQ> ctrl = std::shared_ptr<cRaptorControllerQ>(new cRaptorControllerQ());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	
	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildRaptorControllerCacla(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cRaptorControllerCacla> ctrl = std::shared_ptr<cRaptorControllerCacla>(new cRaptorControllerCacla());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildRaptorControllerMACE(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cRaptorControllerMACE> ctrl = std::shared_ptr<cRaptorControllerMACE>(new cRaptorControllerMACE());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	
	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildBipedController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cBipedControllerQ> ctrl = std::shared_ptr<cBipedControllerQ>(new cBipedControllerQ());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	
	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildBipedController2D(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cBipedController2DQ> ctrl = std::shared_ptr<cBipedController2DQ>(new cBipedController2DQ());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	
	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildBipedController2DCACLA(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cBipedController2DCACLA> ctrl = std::shared_ptr<cBipedController2DCACLA>(new cBipedController2DCACLA());
	// ctrl->setRelativeFilePath(params.mRelativeFilePath);
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);
    if(params.mNumGroundSamples >= 0)
    {
        ctrl->SetNumGroundSamples(params.mNumGroundSamples);
    }
    if(params.mViewDist >= 0)
    {
        ctrl->SetViewDist(params.mViewDist);
    }
	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildBipedController2DCaclaFD(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cBipedController2DCaclaFD> ctrl = std::shared_ptr<cBipedController2DCaclaFD>(new cBipedController2DCaclaFD());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];
	const std::string& fd_net_file = params.mNetFiles[eNetFileForwardDynamics];
	const std::string& fd_model_file = params.mNetFiles[eNetFileForwardDynamicsModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	if (fd_net_file != "")
	{
		
		bool fd_succ = ctrl->LoadForwardDynamicsNet(fd_net_file);
		succ &= fd_succ;
		if (fd_succ && (fd_model_file != ""))
		{
			ctrl->LoadForwardDynamicsModel(fd_model_file);
		}
		
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildBipedController2DParameterized(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cBipedController2DParameterized> ctrl = std::shared_ptr<cBipedController2DParameterized>(new cBipedController2DParameterized());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

/*
// Should generate a controller that does not learn...
bool cTerrainRLCtrlFactory::BuildBipedController3D(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cBipedController3D> ctrl = std::shared_ptr<cBipedController3D>(new cBipedController3D());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}
*/

bool cTerrainRLCtrlFactory::BuildBipedController3DCACLA(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cBipedController3DCACLA> ctrl = std::shared_ptr<cBipedController3DCACLA>(new cBipedController3DCACLA());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtController> ctrl = std::shared_ptr<cCtController>(new cCtController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get());
	ctrl->SetUpdatePeriod(update_period);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtRNNController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtRNNController> ctrl = std::shared_ptr<cCtRNNController>(new cCtRNNController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get());
	ctrl->SetUpdatePeriod(update_period);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtPDController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtPDController> ctrl = std::shared_ptr<cCtPDController>(new cCtPDController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);
	ctrl->SetUpdatePeriod(update_period);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtVelController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtVelController> ctrl = std::shared_ptr<cCtVelController>(new cCtVelController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);
	ctrl->SetUpdatePeriod(update_period);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtNPDController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtNPDController> ctrl = std::shared_ptr<cCtNPDController>(new cCtNPDController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get());
	ctrl->SetUpdatePeriod(update_period);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtMTUController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtMTUController> ctrl = std::shared_ptr<cCtMTUController>(new cCtMTUController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mCtrlParamFile);
	ctrl->SetUpdatePeriod(update_period);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtTrackController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtTrackController> ctrl = std::shared_ptr<cCtTrackController>(new cCtTrackController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get());
	ctrl->SetUpdatePeriod(update_period);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtPDTrackController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtPDTrackController> ctrl = std::shared_ptr<cCtPDTrackController>(new cCtPDTrackController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);
	ctrl->SetUpdatePeriod(update_period);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtVelTrackController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtVelTrackController> ctrl = std::shared_ptr<cCtVelTrackController>(new cCtVelTrackController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);
	ctrl->SetUpdatePeriod(update_period);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtNPDTrackController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtNPDTrackController> ctrl = std::shared_ptr<cCtNPDTrackController>(new cCtNPDTrackController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get());
	ctrl->SetUpdatePeriod(update_period);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtMTUTrackController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtMTUTrackController> ctrl = std::shared_ptr<cCtMTUTrackController>(new cCtMTUTrackController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mCtrlParamFile);
	ctrl->SetUpdatePeriod(update_period);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtPhaseController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtPhaseController> ctrl = std::shared_ptr<cCtPhaseController>(new cCtPhaseController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get());
	ctrl->SetUpdatePeriod(update_period);
	ctrl->SetCycleDur(params.mCycleDur);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtPDPhaseController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtPDPhaseController> ctrl = std::shared_ptr<cCtPDPhaseController>(new cCtPDPhaseController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);
	ctrl->SetUpdatePeriod(update_period);
	ctrl->SetCycleDur(params.mCycleDur);
    if(params.mNumGroundSamples >= 0)
    {
        ctrl->SetNumGroundSamples(params.mNumGroundSamples);
    }
    if(params.mViewDist >= 0)
    {
        ctrl->SetViewDist(params.mViewDist);
    }
	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtVelPhaseController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtVelPhaseController> ctrl = std::shared_ptr<cCtVelPhaseController>(new cCtVelPhaseController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);
	ctrl->SetUpdatePeriod(update_period);
	ctrl->SetCycleDur(params.mCycleDur);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtQController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtQController> ctrl = std::shared_ptr<cCtQController>(new cCtQController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get());
	ctrl->SetUpdatePeriod(update_period);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtQPDController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtQPDController> ctrl = std::shared_ptr<cCtQPDController>(new cCtQPDController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);
	ctrl->SetUpdatePeriod(update_period);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtQVelController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtQVelController> ctrl = std::shared_ptr<cCtQVelController>(new cCtQVelController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);
	ctrl->SetUpdatePeriod(update_period);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtQTrackController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtQTrackController> ctrl = std::shared_ptr<cCtQTrackController>(new cCtQTrackController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get());
	ctrl->SetUpdatePeriod(update_period);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtQPDTrackController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtQPDTrackController> ctrl = std::shared_ptr<cCtQPDTrackController>(new cCtQPDTrackController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);
	ctrl->SetUpdatePeriod(update_period);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtQVelTrackController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtQVelTrackController> ctrl = std::shared_ptr<cCtQVelTrackController>(new cCtQVelTrackController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);
	ctrl->SetUpdatePeriod(update_period);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtTargetController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtTargetController> ctrl = std::shared_ptr<cCtTargetController>(new cCtTargetController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get());
	ctrl->SetUpdatePeriod(update_period);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}


bool cTerrainRLCtrlFactory::BuildCtPDTrackTargetController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtPDTrackTargetController> ctrl = std::shared_ptr<cCtPDTrackTargetController>(new cCtPDTrackTargetController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);
	ctrl->SetUpdatePeriod(update_period);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtPDPhaseTargetController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	auto ctrl = std::shared_ptr<cCtPDPhaseTargetController>(new cCtPDPhaseTargetController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);
	ctrl->SetUpdatePeriod(update_period);
	ctrl->SetCycleDur(params.mCycleDur);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildWaypointController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cCharController> llc;
	{
		BuildBipedStepController3D(params, llc);
	}
	
	auto step_ctrl = std::dynamic_pointer_cast<cBipedStepController3D>(llc);
	auto ctrl = std::shared_ptr<cWaypointController>(new cWaypointController());

	ctrl->SetGround(params.mGround);
	ctrl->EnableSymmetricStep(params.mEnableSymmetricStep);
	ctrl->SetInitStepLen(params.mWaypointInitStepLen);
	ctrl->Init(params.mChar.get());
	ctrl->SetLLC(step_ctrl);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor1];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActor1Model];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic1];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCritic1Model];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}


bool cTerrainRLCtrlFactory::BuildWaypointVelController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cCharController> llc;
	{
		BuildBipedStepController3D(params, llc);
	}

	auto step_ctrl = std::dynamic_pointer_cast<cBipedStepController3D>(llc);
	auto ctrl = std::shared_ptr<cWaypointVelController>(new cWaypointVelController());
	
	ctrl->SetGround(params.mGround);
	ctrl->EnableSymmetricStep(params.mEnableSymmetricStep);
	ctrl->SetInitStepLen(params.mWaypointInitStepLen);
	ctrl->Init(params.mChar.get());
	ctrl->SetLLC(step_ctrl);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor1];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActor1Model];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic1];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCritic1Model];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildSoccerController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cCharController> llc;
	{
		BuildBipedStepController3D(params, llc);
	}

	auto step_ctrl = std::dynamic_pointer_cast<cBipedStepController3D>(llc);
	auto ctrl = std::shared_ptr<cSoccerController>(new cSoccerController());
	
	ctrl->SetGround(params.mGround);
	ctrl->EnableSymmetricStep(params.mEnableSymmetricStep);
	ctrl->SetInitStepLen(params.mWaypointInitStepLen);
	ctrl->Init(params.mChar.get());
	ctrl->SetLLC(step_ctrl);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor1];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActor1Model];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic1];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCritic1Model];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildBipedStepController3D(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	auto ctrl = std::shared_ptr<cBipedStepController3D>(new cBipedStepController3D());
	ctrl->SetGround(params.mGround);
    if(params.mGroundSampleRes3d >= 0)
    {
	    ctrl->SetGroundSampleRes(params.mGroundSampleRes3d);
    }
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);
	ctrl->SetUpdatePeriod(update_period);
	ctrl->SetCycleDur(params.mCycleDur);
    if(params.mGroundSampleRes3d >= 0)
    {
	    ctrl->SetGroundSampleRes(params.mGroundSampleRes3d);
    }
	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtTargetTerrController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	auto ctrl = std::shared_ptr<cCtTargetTerrController>(new cCtTargetTerrController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);
	ctrl->SetUpdatePeriod(update_period);
	ctrl->SetCycleDur(params.mCycleDur);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtTargetSoccerController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	auto ctrl = std::shared_ptr<cCtTargetSoccerController>(new cCtTargetSoccerController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);
	ctrl->SetUpdatePeriod(update_period);
	ctrl->SetCycleDur(params.mCycleDur);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}
