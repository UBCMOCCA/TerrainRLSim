#pragma once

#include "TerrainRLCharController.h"

class cMixController : public cCharController
{
public:
	cMixController();
	virtual ~cMixController();

	virtual void Init(cSimCharacter* character);
	virtual void Reset();
	virtual void Clear();

	virtual int AddController(const std::shared_ptr<cTerrainRLCharController>& ctrl);

	virtual void Update(double time_step);
	virtual void UpdateCalcTau(double time_step, Eigen::VectorXd& out_tau);
	virtual void UpdateApplyTau(const Eigen::VectorXd& tau);
	virtual const Eigen::VectorXd& GetTau() const;
	virtual const Eigen::VectorXd& GetTau(int id) const;

	virtual int GetNumControllers() const;
	virtual const std::shared_ptr<cTerrainRLCharController>& GetController(int id) const;
	virtual void SetWeight(int id, double w);

	virtual bool NewActionUpdate() const;

	virtual int GetState() const;
	virtual double GetPhase() const;
	virtual void SetPhase(double phase);
	virtual int GetNumStates() const;
	virtual double CalcNormPhase() const;

	virtual void TransitionState(int state);
	virtual void TransitionState(int state, double phase);
	virtual bool IsNewCycle() const;

	virtual void CommandAction(int action_id);
	virtual void CommandRandAction();
	virtual int GetDefaultAction() const;
	virtual void SetDefaultAction(int action_id);
	virtual int GetNumActions() const;
	virtual int GetCurrActionID() const;

	virtual void EnableExp(bool enable);
	virtual const tExpParams& GetExpParams() const;
	virtual void SetExpParams(const tExpParams& params);

	virtual double GetViewDist() const;
	virtual void SetViewDist(double dist);

	virtual void BuildNormPose(Eigen::VectorXd& pose) const;

	virtual void BuildFromMotion(int ctrl_params_idx, const cMotion& motion);
	virtual void BuildCtrlOptParams(int ctrl_params_idx, Eigen::VectorXd& out_params) const;
	virtual void SetCtrlOptParams(int ctrl_params_idx, const Eigen::VectorXd& params);
	virtual void BuildActionOptParams(int action_id, Eigen::VectorXd& out_params) const;

	virtual int GetNumGroundSamples() const;
	virtual tVector GetGroundSample(int s) const;
	virtual tMatrix GetGroundSampleTrans() const;

	virtual void BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	
protected:
	std::vector<Eigen::VectorXd> mTaus;
	Eigen::VectorXd mWeights;
	std::vector<std::shared_ptr<cTerrainRLCharController>> mControllers;
	virtual const std::shared_ptr<cTerrainRLCharController>& GetDefaultController() const;
};
