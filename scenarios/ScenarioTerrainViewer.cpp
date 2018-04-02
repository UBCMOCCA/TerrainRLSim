#include "ScenarioTerrainViewer.h"

const double gViewBoundSize = 2;

cScenarioTerrainViewer::cScenarioTerrainViewer()
{
	mTargetCharPos.setZero();
}

cScenarioTerrainViewer::~cScenarioTerrainViewer()
{
}

void cScenarioTerrainViewer::Reset()
{
	cScenarioSimChar::Reset();
	mTargetCharPos.setZero();
}

void cScenarioTerrainViewer::SetTargetCharPos(const tVector& pos)
{
	mTargetCharPos = pos;
}

std::string cScenarioTerrainViewer::GetName() const
{
	return "Terrain Viewer";
}

void cScenarioTerrainViewer::UpdateCharacter(double time_step)
{
	MoveCharacter(mTargetCharPos);
}

void cScenarioTerrainViewer::MoveCharacter(const tVector& pos)
{
	Eigen::VectorXd pose = mChar->GetPose0();
	const auto& joint_mat = mChar->GetJointMat();
	tVector root_pos = cKinTree::GetRootPos(joint_mat, pose);

	tVector origin_pos = pos;
	origin_pos[1] = mGround->SampleHeight(pos);
	origin_pos[1] += root_pos[1];
	
	cKinTree::SetRootPos(joint_mat, origin_pos, pose);
	mChar->SetPose(pose);

	ResolveCharGroundIntersect(mChar);
}

void cScenarioTerrainViewer::GetViewBound(tVector& out_min, tVector& out_max) const
{
	out_min = mTargetCharPos - tVector(gViewBoundSize, 0, gViewBoundSize, 0);
	out_max = mTargetCharPos + tVector(gViewBoundSize, 0, gViewBoundSize, 0);
}