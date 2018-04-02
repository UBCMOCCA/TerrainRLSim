#pragma once

#include "sim/CtController.h"
#include "sim/MusculotendonUnit.h"

class cCtMTUController : public virtual cCtController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cCtMTUController();
	virtual ~cCtMTUController();

	virtual void Init(cSimCharacter* character, const std::string& param_file);
	virtual void Reset();
	virtual void Clear();
	virtual bool LoadParams(const std::string& param_file);

	virtual bool LoadMTUs(const std::string& param_file);
	virtual int GetNumMTUs() const;
	virtual const cMusculotendonUnit& GetMTU(int id) const;
	virtual void HandlePoseReset();
	virtual void HandleVelReset();

	virtual void UpdateCalcTau(double time_step, Eigen::VectorXd& out_tau);

	virtual int GetPoliStateSize() const;
	virtual int GetPoliActionSize() const;
	virtual void BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	
	virtual int GetNumOptParams() const;
	virtual int GetNumOptParams(int mtu_id) const;
	virtual void FetchOptParamScale(Eigen::VectorXd& out_scale) const;
	virtual void BuildOptParams(Eigen::VectorXd& out_params) const;
	virtual void SetOptParams(const Eigen::VectorXd& opt_params);
	virtual int CalcOptParamOffset(int mtu_id) const;

	virtual void OutputOptParams(FILE* f, const Eigen::VectorXd& params) const;
	virtual std::string BuildOptParamsJson(const Eigen::VectorXd& params) const;

protected:
	std::vector<cMusculotendonUnit> mMTUs;

	virtual void ResetMTUs();
	virtual void ResetCEState();

	virtual void SetupActionBounds();
	virtual void UpdateMTUs(double time_step, Eigen::VectorXd& out_tau);
	virtual void ApplyAction(const tAction& action);

	virtual int GetMTUStateSize() const;
	virtual int GetMTUStateOffset() const;
	virtual void BuildPoliState(Eigen::VectorXd& out_state) const;
	virtual void BuildMTUState(Eigen::VectorXd& out_state) const;

	virtual void ApplyExpNoise(tAction& out_action);
};
