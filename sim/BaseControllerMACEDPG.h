#pragma once

#include "sim/BaseControllerDPG.h"

class cBaseControllerMACEDPG : public virtual cBaseControllerDPG
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cBaseControllerMACEDPG();
	virtual ~cBaseControllerMACEDPG();

	virtual void Reset();

	virtual int GetNumActionFrags() const;
	virtual int GetActionFragSize() const;
	virtual int GetNetOutputSize() const;

	virtual int GetActorOutputSize() const;
	virtual int GetCriticInputSize() const;

	virtual void BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;

protected:
	int mNumActionFrags;
	Eigen::VectorXd mBoltzmannBuffer;
	bool mExpCritic;
	bool mExpActor;

	virtual void LoadNetIntern(const std::string& net_file);
	virtual void UpdateFragParams();

	virtual void ProcessCommand(tAction& out_action);

	virtual void BuildActionFragOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildActorBias(int a_id, Eigen::VectorXd& out_bias) const = 0;
	
	virtual void CalcCriticVals(const Eigen::VectorXd& state, const Eigen::VectorXd& actions, Eigen::VectorXd& out_vals);
	virtual void BuildActorAction(const Eigen::VectorXd& actions, int a_id, tAction& out_action) const;

	virtual void UpdateAction();
	virtual void DecideAction(tAction& out_action);
	virtual void ExploitPolicy(const Eigen::VectorXd& state, tAction& out_action);
	virtual void DecideActionBoltzmann(tAction& out_action);
	virtual int BoltzmannSelectActor(const Eigen::VectorXd& vals, Eigen::VectorXd& val_buffer) const;
	
	virtual int GetMaxValIdx(const Eigen::VectorXd& vals) const;
	virtual double GetMaxVal(const Eigen::VectorXd& vals) const;
	virtual void GetFrag(const Eigen::VectorXd& actions, int a_idx, Eigen::VectorXd& out_action) const;
	virtual void SetFrag(const Eigen::VectorXd& actions, int a_idx, Eigen::VectorXd& out_actions) const;
	virtual double GetVal(const Eigen::VectorXd& vals, int a_idx) const;
	virtual void SetVal(double val, int a_idx, Eigen::VectorXd& out_vals) const;

#if defined (ENABLE_DEBUG_PRINT)
	virtual void PrintCriticVals(const Eigen::VectorXd& vals, int a_id) const;
#endif

#if defined(ENABLE_DEBUG_VISUALIZATION)
public:
	virtual void GetVisActionFeatures(Eigen::VectorXd& out_features) const;
	virtual void GetVisActionValues(Eigen::VectorXd& out_vals) const;

protected:
#endif
};