#pragma once

#include "scenarios/ScenarioExp.h"
#include "sim/TerrainRLCharController.h"
#include "anim/KinCharacter.h"

class cScenarioExpMimic : virtual public cScenarioExp
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum eMimicMode
	{
		eMimicModeTau,
		eMimicModeAction,
		eMimicModeMax
	};

	cScenarioExpMimic();
	virtual ~cScenarioExpMimic();

	virtual void Reset();
	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Init();

	virtual const std::shared_ptr<cKinCharacter>& GetKinChar() const;

	virtual void EnableCoachCtrl(bool enable);
	virtual void SetCoachBlend(double blend);
	virtual void EnableCoachActiveProb(bool enable);
	virtual void SetCoachActiveProb(double prob);
	virtual int GetCharCtrlID() const;
	virtual int GetCoachCtrlID() const;

	virtual std::string GetName() const;

	virtual double CalcReward() const;

protected:

	eMimicMode mMimicMode;
	std::string mMotionFile;
	std::shared_ptr<cKinCharacter> mKinChar;

	cTerrainRLCtrlFactory::tCtrlParams mCoachCtrlParams;
	std::string mCoachPoliNetFile;
	std::string mCoachPoliModelFile;
	std::string mCoachCriticNetFile;
	std::string mCoachCriticModelFile;

	Eigen::VectorXd mCoachAction;
	Eigen::VectorXd mCoachActionSample;
	int mCoachActionCount;

	int mCharCtrlID;
	int mCoachCtrlID;

	double mCoachBlend;
	double mCoachActiveProb;
	bool mEnableCoachActiveProb;

	double mCoachWarmupMax;
	double mCouchWarmupTimeMax;
	double mCoachWarmupTimer;

	virtual void ParseMimicMode(const std::string& str, eMimicMode& out_mode) const;
	virtual bool BuildKinCharacter(std::shared_ptr<cKinCharacter>& out_char) const;

	virtual bool BuildController(std::shared_ptr<cCharController>& out_ctrl);
	virtual void SetupCoachControllerParams(cTerrainRLCtrlFactory::tCtrlParams& out_params) const;
	virtual bool BuildCoachController(const cTerrainRLCtrlFactory::tCtrlParams& params, std::shared_ptr<cTerrainRLCharController>& out_ctrl);
	virtual bool EnableRandInitAction() const;
	virtual void ApplyCoachBlend(double blend);

	virtual void UpdateCharacter(double time_step);
	virtual void UpdateKinChar(double time_step);
	virtual void UpdateTrackController();
	virtual void FetchCoachAction(double time_step, Eigen::VectorXd& out_action);
	virtual void HandleNewActionUpdate();
	virtual void NewActionUpdateCoach();
	virtual void PreSubstepUpdate(double time_step);
	virtual void PostSubstepUpdate(double time_step);

	virtual int GetNumWarmupCycles() const;
	virtual void IncCycleCount();

	virtual std::shared_ptr<const cNNController> GetNNController() const;
	virtual void RecordAction(Eigen::VectorXd& out_action) const;

	virtual bool IsCoachWarmup() const;
	virtual void ResetCoachWarmupCounter();
	virtual void UpdateCoachWarmupCounter(double time_step);

	virtual double GetEpisodeMaxTime() const;
};
