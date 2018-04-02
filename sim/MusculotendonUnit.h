#pragma once

#include "util/json/json.h"
#include "sim/Controller.h"
#include "sim/Joint.h"

class cMusculotendonUnit : public cController
{
public:

	enum eOptParam
	{
		eOptParamOptCELength,
		eOptParamSlackLength,
		eOptParamForceMax,
		eOptParamMax
	};

	enum eOptSegParam
	{
		eOptSegParamArm,
		eOptSegParamThetaMax,
		eOptSegParamThetaRest,
		eOptSegParamMax
	};

	struct tAttachPt
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		int mJointID;
		tVector mLocalPos;

		bool mFixedArm;
		double mArm;
		double mThetaMax;
		double mThetaRest;
		double mPennation;

		tAttachPt();
		bool Parse(const Json::Value& json);
		std::string BuildJson() const;
	};

	struct tParams
	{
		int mID;
		std::string mName;
		double mOptCELength;
		double mSlackLength;
		double mForceMax;
		double mPennation;
		std::vector<tAttachPt, Eigen::aligned_allocator<tAttachPt>> mAttachPts;

		tParams();
		void Clear();
	};

	struct tParamData
	{
		double mScale;
		double mValMin;
		double mValMax;
	};
	
	static const double gMinActivation;
	static const double gMaxActivation;

	static bool ParseParams(const Json::Value& json, tParams& out_params);
	static double GetRefStrain();

	cMusculotendonUnit();
	virtual ~cMusculotendonUnit();

	virtual void Init(cSimCharacter* character, const tParams& params);
	virtual void Clear();
	virtual void Reset();
	virtual void Update(double time_step);

	virtual void UpdateCalcTau(double time_step, Eigen::VectorXd& out_tau);
	virtual void UpdateApplyTau(const Eigen::VectorXd& tau);
	
	virtual int GetNumAttachPts() const;
	virtual const tAttachPt& GetAttachPt(int id) const;
	virtual tAttachPt& GetAttachPt(int id);
	virtual tVector CalcAttachPtWorldPos(int id) const;
	virtual double CalcLength() const;
	virtual double GetCELength() const;
	virtual double GetOptCELength() const;
	virtual double GetSlackLength() const;
	virtual double CalcCEStrain() const;
	virtual double CalcSEStrain() const;

	virtual double GetActivation() const;
	virtual void SetExcitation(double u);
	virtual double GetExcitation() const;

	virtual void ResetCELength();

	virtual int GetNumOptParams() const;
	virtual void FetchOptParamScale(Eigen::VectorXd& out_scale) const;
	virtual void BuildOptParams(Eigen::VectorXd& out_params) const;
	virtual void SetOptParams(const Eigen::VectorXd& opt_params);

	virtual std::string BuildOptParamsJson(const Eigen::VectorXd& params) const;

protected:

	struct tSegActuation
	{
		int mSegID;
		std::vector<int> mJointIDs;
	};

	tParams mParams;
	std::vector<tSegActuation> mSegActuations; // records which joints are actuated by which segments

	double mActivationRate;
	double mActivation;
	double mExcitation;
	double mCEVel;
	double mCELength;
	double mForce;

	virtual void ResetParams();

	virtual int GetNumSegs() const;
	virtual void SetupSegActuations(std::vector<tSegActuation>& out_joint_segs) const;
	virtual double GetRestLength() const;

	virtual double CalcForceSerial(double serial_len) const;
	virtual double CalcForceParallel(double parallel_len) const;
	virtual double CalcForceElasticBuffer(double ce_len) const;
	virtual double CalcForceVel(double vel_ce) const;
	virtual double CalcForceLengthRel(double ce_len) const;
	virtual double CalcForceParallelRel(double ce_len) const;
	virtual double CalcVelCE(double fv) const;

	virtual double CalcLengthIntern() const;
	virtual double CalcSerialLen() const;

	virtual void UpdateActivation(double time_step);
	virtual void ApplyForce(double force, Eigen::VectorXd& out_tau) const;

	virtual void PostProcessParams(Eigen::VectorXd& out_params) const;
};
