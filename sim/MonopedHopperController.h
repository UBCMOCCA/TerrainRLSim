#pragma once

#include "sim/PDController.h"
#include "sim/ImpPDController.h"
#include "sim/SimMonopedHopper.h"
#include "sim/RBDModel.h"
#include "sim/TerrainRLCharController.h"

class cMonopedHopperController : public virtual cTerrainRLCharController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum eStateParam
	{
		eStateParamHip,
		eStateParamKnee,
		eStateParamMax
	};
	typedef Eigen::Matrix<double, eStateParamMax, 1> tStateParams;

	enum eMiscParam
	{
		eMiscParamMagicGain,
		eMiscParamMax
	};
	typedef Eigen::Matrix<double, eMiscParamMax, 1> tMiscParams;

	enum eState
	{
		eStateDescent,
		eStateDownStance,
		eStateUpStance,
		eStateAscent,
		eStateMax,
		eStateInvalid,
	};

	struct tStateDef
	{
		std::string mName;
		bool mTransTime;
		cSimMonopedHopper::eJoint mTransContact;
		eState mNext;
	};

	cMonopedHopperController();
	virtual ~cMonopedHopperController();

	virtual void Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file);
	virtual void Reset();
	virtual void Clear();

	virtual void Update(double time_step);
	virtual void UpdateCalcTau(double time_step, Eigen::VectorXd& out_tau);
	virtual void UpdateApplyTau(const Eigen::VectorXd& tau);

	virtual void SeCtrlStateParams(eState state, const tStateParams& params);
	
	virtual void TransitionState(int state);
	virtual void TransitionState(int state, double phase);
	virtual int GetNumStates() const;

	virtual void SetMode(eMode mode);
	virtual void CommandAction(int action_id);
	virtual void CommandRandAction();
	virtual int GetDefaultAction() const;
	virtual void SetDefaultAction(int action_id);
	virtual int GetNumActions() const;

	virtual void BuildCtrlOptParams(int ctrl_params_id, Eigen::VectorXd& out_params) const;
	virtual void SetCtrlParams(int ctrl_params_id, const Eigen::VectorXd& params);
	virtual void SetCtrlOptParams(int ctrl_params_id, const Eigen::VectorXd& params);
	virtual void BuildActionOptParams(int action_id, Eigen::VectorXd& out_params) const;
	virtual std::string BuildOptParamsJson(const Eigen::VectorXd& opt_params) const;
	virtual std::string BuildParamsJson(const Eigen::VectorXd& params) const;
	virtual void DebugPrintAction(const tAction& action) const;
	virtual int GetNumParams() const;
	virtual int GetNumOptParams() const;
	virtual void FetchOptParamScale(Eigen::VectorXd& out_scale) const;
	virtual void OutputOptParams(const std::string& file, const Eigen::VectorXd& params) const;
	virtual void OutputOptParams(FILE* f, const Eigen::VectorXd& params) const;

	virtual void SetParams(const Eigen::VectorXd& params);
	virtual void BuildOptParams(Eigen::VectorXd& out_params) const;
	virtual void SetOptParams(const Eigen::VectorXd& params);
	virtual void SetOptParams(const Eigen::VectorXd& opt_params, Eigen::VectorXd& out_params) const;

	virtual void ReadParams(const std::string& file);
	virtual void ReadParams(std::ifstream& f_stream);

	virtual void BuildFromMotion(int ctrl_params_id, const cMotion& motion);

	virtual double CalcReward() const;

	virtual double GetPrevCycleTime() const;
	virtual const tVector& GetPrevDistTraveled() const;
	
protected:
	struct tBlendAction
	{
		int mID;
		int mParamIdx0;
		int mParamIdx1;
		double mBlend;
		bool mCyclic;
	};

	tVector mGravity;
	cImpPDController mImpPDCtrl;
	// cExpPDController mImpPDCtrl;

	std::shared_ptr<cRBDModel> mRBDModel;
	Eigen::MatrixXd mJacobian;

	std::vector<Eigen::VectorXd> mCtrlParams;
	int mDefaultAction;
	std::vector<tBlendAction> mActions;
	std::stack<int> mCommands;
	bool mEnableGravityCompensation;

	Eigen::VectorXd _errors;
	Eigen::VectorXd _desired_angles;

	double mPrevCycleTime;
	double mCurrCycleTime;
	tVector mPrevCOM;
	tVector mPrevDistTraveled;
	double mPrevStumbleCount;
	double mCurrStumbleCount;
	double _previousBody_Y;

	virtual void ResetParams();

	virtual bool LoadControllers(const std::string& file);
	virtual bool ParseControllers(const Json::Value& root);
	virtual bool ParseControllerFiles(const Json::Value& root);
	virtual bool ParseActions(const Json::Value& root);
	virtual bool ParseAction(const Json::Value& root, tBlendAction& out_action) const;
	virtual double GetRootPitch() const;

	virtual bool HasCtrlParams() const;
	virtual void UpdateState(double time_step);
	virtual void UpdateAction();
	virtual void UpdateRBDModel();
	virtual void UpdatePDCtrls(double time_step);
	virtual void UpdateStumbleCounter(double time_step);
	virtual void ApplyFeedback(Eigen::VectorXd& out_tau);
	virtual void ApplyGravityCompensation(Eigen::VectorXd& out_tau);
	virtual void ApplyVirtualForces(Eigen::VectorXd& out_tau);

	virtual const tStateDef& GetCurrStateDef() const;
	virtual tStateParams GetCurrParams() const;

	virtual void SetStateParams(const tStateParams& params);
	virtual void SetupPassiveMode();

	virtual bool CheckContact(cSimMonopedHopper::eJoint joint_id) const;
	virtual bool IsActiveVFEffector(cSimMonopedHopper::eJoint joint_id) const;
	virtual tVector GetEffectorVF(cSimMonopedHopper::eJoint joint_id) const;
	virtual Eigen::MatrixXd BuildContactBasis(const Eigen::VectorXd& pose, bool& out_has_support) const;

	const std::string& GetStateName(eState state) const;
	const std::string& GetStateParamName(eStateParam param) const;

	virtual void GetOptParams(const Eigen::VectorXd& ctrl_params, Eigen::VectorXd& out_opt_params) const;
	// std::string BuildOptParamsJson(const Eigen::VectorXd& params) const;
	virtual void BuildStateParamsFromPose(const Eigen::VectorXd& pose, tStateParams& out_params);

	virtual const Eigen::VectorXd& GetCtrlParams(int ctrl_id) const;

	virtual bool IsCurrActionCyclic() const;
	virtual void ApplyAction(int action_id);
	virtual void ApplyAction(const tAction& action);
	virtual void NewCycleUpdate();
	virtual void BlendCtrlParams(const tBlendAction& action, Eigen::VectorXd& out_params) const;
	virtual void PostProcessParams(Eigen::VectorXd& out_params) const;
	virtual void PostProcessAction(tAction& out_action) const;
	virtual bool IsOptParam(int param_idx) const;

	virtual tVector GetEndEffectorContactPos(int joint_id) const;
	virtual void RecordDistTraveled();

	virtual int PopCommand();
	virtual bool HasCommands() const;
	virtual void ClearCommands();
	virtual void ProcessCommand(tAction& out_action);

	virtual void BuildPoliStatePose(Eigen::VectorXd& out_pose) const;
	virtual void BuildPoliStateVel(Eigen::VectorXd& out_vel) const;
	virtual int GetPoliStateSize() const;
	virtual int GetPoliStateSize(ePoliState params) const;

	virtual void BuildBaseAction(int action_id, tAction& out_action) const;
	virtual double GetGetMagicGain() const;
};
