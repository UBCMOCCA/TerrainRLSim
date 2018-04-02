#include "ScenarioHackMultChar.h"

cScenarioHackMultChar::cScenarioHackMultChar()
{
}

cScenarioHackMultChar::~cScenarioHackMultChar()
{
}

void cScenarioHackMultChar::Init()
{
	cScenarioPoliEval::Init();
	BuildChar1();
}

const std::shared_ptr<cSimCharacter>& cScenarioHackMultChar::GetHackCharacter() const
{
	return mChar1;
}

std::string cScenarioHackMultChar::GetName() const
{
	return "Hack Multiple Characters";
}

void cScenarioHackMultChar::BuildChar1()
{
	CreateCharacter(eCharRaptor, mChar1);

	cSimCharacter::tParams char_params;
	char_params.mInitPos = GetDefaultCharPos();
	char_params.mCharFile = "data/characters/raptor.txt";
	char_params.mStateFile = "data/states/raptor_run_state2.txt";

	bool succ = mChar1->Init(mWorld, char_params);
	if (succ)
	{
		mChar1->RegisterContacts(cWorld::eContactFlagCharacter, cWorld::eContactFlagEnvironment);
		InitCharacterPos(mChar1);

		std::shared_ptr<cCharController> ctrl;
		cTerrainRLCtrlFactory::tCtrlParams ctrl_params;
		ctrl_params.mCharCtrl = cTerrainRLCtrlFactory::eCharCtrlRaptorMACE;
		ctrl_params.mCtrlParamFile = "data/characters/raptor.txt";
		ctrl_params.mChar = mChar1;
		ctrl_params.mGravity = GetGravity();
		ctrl_params.mGround = mGround;

		ctrl_params.mNetFiles = mCtrlParams.mNetFiles;
		ctrl_params.mNetFiles[cTerrainRLCtrlFactory::eNetFileActor] = "data/policies/raptor/nets/raptor_mace3_deploy.prototxt";
		ctrl_params.mNetFiles[cTerrainRLCtrlFactory::eNetFileActorModel] = "data/policies/raptor/models/raptor_mace3_slopes_mixed_model.h5";

		bool succ = cTerrainRLCtrlFactory::BuildController(ctrl_params, ctrl);

		if (succ && ctrl != nullptr)
		{
			mChar1->SetController(ctrl);
		}
	}

	tVector root_pos = mChar->GetRootPos();
	root_pos += tVector(0, 0, -0.5, 0);
	mChar->SetRootPos(root_pos);
	mChar->SetRootPos0(root_pos);

	tVector root_pos1 = mChar1->GetRootPos();
	root_pos1 += tVector(0, 0, 0.5, 0);
	mChar1->SetRootPos(root_pos1);
	mChar1->SetRootPos0(root_pos1);
}

void cScenarioHackMultChar::UpdateCharacter(double time_step)
{
	cScenarioPoliEval::UpdateCharacter(time_step);
	mChar1->Update(time_step);
}

void cScenarioHackMultChar::Reset()
{
	cScenarioPoliEval::Reset();

	mChar1->Reset();
	InitCharacterPos(mChar1);
	ResolveCharGroundIntersect(mChar1);
}