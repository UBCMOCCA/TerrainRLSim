#pragma once
#include <memory>

#include "DrawScenarioTerrainRL.h"
#include "ScenarioKinMotion.h"

class cDrawScenarioKinMotion : public cDrawScenarioTerrainRL
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioKinMotion(cCamera& cam);
	virtual ~cDrawScenarioKinMotion();

	virtual void Init();
	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Keyboard(unsigned char key, int x, int y);

	virtual void Reset();
	virtual void Clear();
	
	virtual std::string BuildTextInfoStr() const;

	std::string GetName() const;

protected:
	cScenarioKinMotion mScene;

	std::string mAnnotateOutputFile;

	virtual void UpdateScene(double time_elapsed);

	virtual tVector GetCamTrackPos() const;
	virtual tVector GetCamStillPos() const;
	virtual tVector GetDefaultCamFocus() const;

	virtual void DrawGround() const;
	virtual void DrawGround2D() const;
	virtual void DrawGround3D() const;
	virtual void DrawCharacters() const;

	virtual bool EnableAnnotation() const;
	virtual void Annotate();
};