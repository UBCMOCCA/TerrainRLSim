#include "MusculotendonUnit.h"
#include "SimCharacter.h"
#include "util/JsonUtil.h"

#define ENABLE_FIXED_ARM
#define ENABLE_ANALYTIC_ARM
#define ENABLE_ACTIVATION_CTRL
//#define ENABLE_SOFT_EXCITATION_CLAMP

const std::string gIDKey = "ID";
const std::string gNameKey = "Name";
const std::string gOptCELengthKey = "OptCELength";
const std::string gSlackLengthKey = "SlackLength";
const std::string gForceMaxKey = "ForceMax";
const std::string gAttachPtsKey = "AttachPts";
const std::string gJointIDKey = "JointID";
const std::string gLocalPosKey = "LocalPos";
const std::string gFixedArmKey = "FixedArm";
const std::string gArmKey = "Arm";
const std::string gThetaMaxKey = "ThetaMax";
const std::string gThetaRestKey = "ThetaRest";
const std::string gPennationKey = "Pennation";
const std::string gFlexJointKey = "FlexJointKey";

const double gMaxVel = -10;
const double gN = 1.5; // eccentric force enhancement
const double gK = 5; // curvature constant
const double gFvMin = 0.0;
const double gFvMax = 1.5;
const double gOmega = 0.56; // force-length decay width
const double gC = 0.05; // force-length
const double gRefStrain = 0.04;
const double gMinLen = 0.001;

// mScale, mValMin, mValMax
const cMusculotendonUnit::tParamData gOptParamData[cMusculotendonUnit::eOptParamMax] =
{
	{ 0.01,	0.01, 0.3},		// eOptParamOptCELength
	{ 0.01, 0.01, 0.5},		// eOptParamSlackLength
	{ 100, -10000, -100 }	// eOptParamForceMax
};

const cMusculotendonUnit::tParamData gOptSegParamData[cMusculotendonUnit::eOptSegParamMax] =
{
	{ 0.01, 0.01, 0.2},				// eOptSegParamArm
	{ M_PI / 20, -M_PI , M_PI },	// eOptSegParamThetaMax
	{ M_PI / 20, -M_PI , M_PI }		// eOptSegParamThetaRest
};

const double cMusculotendonUnit::gMinActivation = 0.01;
const double cMusculotendonUnit::gMaxActivation = 1;

cMusculotendonUnit::tAttachPt::tAttachPt()
{
	mJointID = gInvalidIdx;
	mLocalPos.setZero();

	mFixedArm = false;
	mArm = 0;
	mThetaMax = 0;
	mThetaRest = 0;
}

bool cMusculotendonUnit::tAttachPt::Parse(const Json::Value& json)
{
	bool succ = true;
	mJointID = json[gJointIDKey].asInt();
	cJsonUtil::ReadVectorJson(json[gLocalPosKey], mLocalPos);

	mFixedArm = json.get(gFixedArmKey, mFixedArm).asBool();
	mArm = json.get(gArmKey, mArm).asDouble();
	mThetaMax = json.get(gThetaMaxKey, mThetaMax).asDouble();
	mThetaRest = json.get(gThetaRestKey, mThetaRest).asDouble();

	return succ;
}

std::string cMusculotendonUnit::tAttachPt::BuildJson() const
{
	std::string json = "{";
	json += "\"" + gJointIDKey + "\": " + std::to_string(mJointID);
	json += ", ";
	json += "\"" + gLocalPosKey + "\": " + cJsonUtil::BuildVectorJson(mLocalPos);

	if (mArm != 0)
	{
		json += ", ";
		json += "\"" + gArmKey + "\": " + std::to_string(mArm);
		json += ", ";

		if (mFixedArm)
		{
			json += "\"" + gFixedArmKey + "\": true";
			json += ", ";
		}
		else
		{
			json += "\"" + gThetaMaxKey + "\": " + std::to_string(mThetaMax);
			json += ", ";
		}

		json += "\"" + gThetaRestKey + "\": " + std::to_string(mThetaRest);
	}

	json += "}";
	return json;
}

cMusculotendonUnit::tParams::tParams()
{
	Clear();
}

void cMusculotendonUnit::tParams::Clear()
{
	mOptCELength = 0;
	mSlackLength = 0;
	mForceMax = 0;
	mPennation = 1;
	mAttachPts.clear();
}

bool cMusculotendonUnit::ParseParams(const Json::Value& json, tParams& out_params)
{
	bool succ = !json.isNull();
	if (succ)
	{
		out_params.Clear();
		out_params.mID = json.get(gIDKey, out_params.mOptCELength).asInt();
		out_params.mName = json.get(gNameKey, out_params.mOptCELength).asString();
		out_params.mOptCELength = json.get(gOptCELengthKey, out_params.mOptCELength).asDouble();
		out_params.mSlackLength = json.get(gSlackLengthKey, out_params.mSlackLength).asDouble();
		out_params.mForceMax = json.get(gForceMaxKey, out_params.mForceMax).asDouble();
		out_params.mPennation = json.get(gPennationKey, out_params.mPennation).asDouble();

		const Json::Value& attach_pt_arr = json[gAttachPtsKey];
		int num_pts = attach_pt_arr.size();
		for (int i = 0; i < num_pts; ++i)
		{
			const Json::Value& attach_pt_json = attach_pt_arr.get(i, 0);
			tAttachPt attach_pt;
			bool pt_succ = attach_pt.Parse(attach_pt_json);

			if (pt_succ)
			{
				out_params.mAttachPts.push_back(attach_pt);
			}
			else
			{
				break;
				succ = false;
			}
		}
		out_params.mAttachPts;
	}

	if (!succ)
	{
		out_params.Clear();
	}

	return succ;
}

double cMusculotendonUnit::GetRefStrain()
{
	return gRefStrain;
}

cMusculotendonUnit::cMusculotendonUnit()
	: cController()
{
	Clear();
	mActivationRate = 100;
}

cMusculotendonUnit::~cMusculotendonUnit()
{
}

void cMusculotendonUnit::Init(cSimCharacter* character, const tParams& params)
{
	cController::Init(character);
	mValid = true;
	mParams = params;
	ResetParams();

	Eigen::VectorXd opt_params;
	BuildOptParams(opt_params);
	PostProcessParams(opt_params);
	SetOptParams(opt_params);

	SetupSegActuations(mSegActuations);
	ResetCELength();
}

void cMusculotendonUnit::Clear()
{
	cController::Clear();
	ResetParams();
	mParams.Clear();
	mSegActuations.clear();
}

void cMusculotendonUnit::Reset()
{
	cController::Reset();
	ResetParams();
	ResetCELength();
}

void cMusculotendonUnit::Update(double time_step)
{
	Eigen::VectorXd tau;
	UpdateCalcTau(time_step, tau);
	UpdateApplyTau(tau);
}

void cMusculotendonUnit::UpdateCalcTau(double time_step, Eigen::VectorXd& out_tau)
{
	int num_dof = mChar->GetNumDof();
	out_tau = Eigen::VectorXd::Zero(num_dof);

	if (IsActive())
	{
		UpdateActivation(time_step);
		
		double serial_len = CalcSerialLen();

		double F_se = CalcForceSerial(serial_len);
		double F_be = CalcForceElasticBuffer(mCELength);

		double fl = CalcForceLengthRel(mCELength);
		double f_pe = CalcForceParallelRel(mCELength);
		double fv = (F_se + F_be) / (mActivation * fl + f_pe);

		double v_ce = CalcVelCE(fv);

		mCEVel = v_ce * mParams.mOptCELength;
		mCELength += time_step * mCEVel;
		mCELength = std::max(gMinLen, mCELength);
		mForce = F_se * mParams.mForceMax;

		ApplyForce(mForce, out_tau);
	}
}

void cMusculotendonUnit::UpdateApplyTau(const Eigen::VectorXd& tau)
{
	mChar->ApplyControlForces(tau);
}

int cMusculotendonUnit::GetNumAttachPts() const
{
	return static_cast<int>(mParams.mAttachPts.size());
}

const cMusculotendonUnit::tAttachPt& cMusculotendonUnit::GetAttachPt(int id) const
{
	assert(id >= 0 && id < GetNumAttachPts());
	return mParams.mAttachPts[id];
}

cMusculotendonUnit::tAttachPt& cMusculotendonUnit::GetAttachPt(int id)
{
	assert(id >= 0 && id < GetNumAttachPts());
	return mParams.mAttachPts[id];
}

tVector cMusculotendonUnit::CalcAttachPtWorldPos(int id) const
{
	const tAttachPt& pt = GetAttachPt(id);
	const cJoint& joint = mChar->GetJoint(pt.mJointID);
	tVector pos = joint.CalcWorldPos(pt.mLocalPos);
	return pos;
}

double cMusculotendonUnit::CalcLength() const
{
	double len = 0;
	int num_pts = GetNumAttachPts();
	if (num_pts > 1)
	{
		tVector prev_pos = CalcAttachPtWorldPos(0);
		for (int i = 1; i < num_pts; ++i)
		{
			tVector curr_pos = CalcAttachPtWorldPos(i);
			len += (curr_pos - prev_pos).norm();
			prev_pos = curr_pos;
		}
	}
	return len;
}

double cMusculotendonUnit::GetCELength() const
{
	return mCELength;
}

double cMusculotendonUnit::GetOptCELength() const
{
	return mParams.mOptCELength;
}

double cMusculotendonUnit::GetSlackLength() const
{
	return mParams.mSlackLength;
}

double cMusculotendonUnit::CalcCEStrain() const
{
	double strain = mCELength / mParams.mOptCELength;
	strain -= 1;
	return strain;
}

double cMusculotendonUnit::CalcSEStrain() const
{
	double serial_len = CalcSerialLen();
	double strain = serial_len / mParams.mSlackLength;
	strain -= 1;
	return strain;
}

double cMusculotendonUnit::GetActivation() const
{
	return mActivation;
}

void cMusculotendonUnit::SetExcitation(double u)
{
	mExcitation = u;
}

double cMusculotendonUnit::GetExcitation() const
{
	return mExcitation;
}

void cMusculotendonUnit::ResetCELength()
{
	double total_len = CalcLengthIntern();
	double slack = mParams.mSlackLength;
	mCELength = total_len - slack;
	mCELength = std::max(gMinLen, mCELength);
	//mCELength = mParams.mOptCELength;
}

int cMusculotendonUnit::GetNumOptParams() const
{
	int num_params = eOptParamMax;
	int num_active_segs = static_cast<int>(mSegActuations.size());
	num_params += num_active_segs * eOptSegParamMax;
	return num_params;
}

void cMusculotendonUnit::FetchOptParamScale(Eigen::VectorXd& out_scale) const
{
	int num_params = GetNumOptParams();
	out_scale.resize(num_params);

	for (int i = 0; i < eOptParamMax; ++i)
	{
		out_scale[i] = gOptParamData[i].mScale;
	}

	int num_active_segs = static_cast<int>(mSegActuations.size());
	for (int s = 0; s < num_active_segs; ++s)
	{
		int offset = eOptParamMax + s * eOptSegParamMax;
		for (int i = 0; i < eOptSegParamMax; ++i)
		{
			out_scale[offset + i] = gOptSegParamData[i].mScale;
		}
	}
}

void cMusculotendonUnit::BuildOptParams(Eigen::VectorXd& out_params) const
{
	int num_params = GetNumOptParams();
	out_params.resize(num_params);

	out_params[eOptParamOptCELength] = mParams.mOptCELength;
	out_params[eOptParamSlackLength] = mParams.mSlackLength;
	out_params[eOptParamForceMax] = mParams.mForceMax;

	int num_active_segs = static_cast<int>(mSegActuations.size());
	for (int s = 0; s < num_active_segs; ++s)
	{
		int offset = eOptParamMax + s * eOptSegParamMax;
		const tSegActuation& seg = mSegActuations[s];
		const tAttachPt& attach_pt = GetAttachPt(seg.mSegID + 1);
		out_params[offset + eOptSegParamArm] = attach_pt.mArm;
		out_params[offset + eOptSegParamThetaMax] = attach_pt.mThetaMax;
		out_params[offset + eOptSegParamThetaRest] = attach_pt.mThetaRest;
	}
}

void cMusculotendonUnit::SetOptParams(const Eigen::VectorXd& opt_params)
{
	Eigen::VectorXd params = opt_params;
	PostProcessParams(params);

	mParams.mOptCELength = params[eOptParamOptCELength];
	mParams.mSlackLength = params[eOptParamSlackLength];
	mParams.mForceMax = params[eOptParamForceMax];

	int num_active_segs = static_cast<int>(mSegActuations.size());
	for (int s = 0; s < num_active_segs; ++s)
	{
		int offset = eOptParamMax + s * eOptSegParamMax;
		const tSegActuation& seg = mSegActuations[s];
		tAttachPt& attach_pt = GetAttachPt(seg.mSegID + 1);

		attach_pt.mArm = params[offset + eOptSegParamArm];
		attach_pt.mThetaMax = params[offset + eOptSegParamThetaMax];
		attach_pt.mThetaRest = params[offset + eOptSegParamThetaRest];
	}
}

std::string cMusculotendonUnit::BuildOptParamsJson(const Eigen::VectorXd& params) const
{
	int num_params = static_cast<int>(params.size());
	assert(num_params == GetNumOptParams());

	Eigen::VectorXd processed_params = params;
	PostProcessParams(processed_params);

	std::string json = "{\n";

	json += "\"" + gIDKey + "\": " + std::to_string(mParams.mID) + ",\n";
	json += "\"" + gNameKey + "\": \"" + mParams.mName + "\",\n";
	json += "\"" + gOptCELengthKey + "\": " + std::to_string(processed_params[eOptParamOptCELength]) + ",\n";
	json += "\"" + gSlackLengthKey + "\": " + std::to_string(processed_params[eOptParamSlackLength]) + ",\n";
	json += "\"" + gForceMaxKey + "\": " + std::to_string(processed_params[eOptParamForceMax]) + ",\n";
	json += "\"" + gPennationKey + "\": " + std::to_string(mParams.mPennation) + ",\n";
	json += "\"" + gAttachPtsKey + "\":\n[\n";

	for (int i = 0; i < GetNumAttachPts(); ++i)
	{
		tAttachPt attach_pt = GetAttachPt(i);

		for (size_t s = 0; s < mSegActuations.size(); ++s)
		{
			const tSegActuation& seg = mSegActuations[s];
			if (seg.mSegID + 1 == i)
			{
				int offset = static_cast<int>(eOptParamMax + s * eOptSegParamMax);
				const tSegActuation& seg = mSegActuations[s];

				attach_pt.mArm = processed_params[offset + eOptSegParamArm];
				attach_pt.mThetaMax = processed_params[offset + eOptSegParamThetaMax];
				attach_pt.mThetaRest = processed_params[offset + eOptSegParamThetaRest];
			}
		}

		if (i != 0)
		{
			json += ",\n";
		}

		std::string attach_json = attach_pt.BuildJson();
		json += attach_json;
	}

	json += "\n]";
	json += "\n}";
	return json;
}

void cMusculotendonUnit::ResetParams()
{
	mActivation = gMinActivation;
	mExcitation = 0;
	mCEVel = 0;
	mCELength = mParams.mOptCELength;
	mForce = 0;
}

int cMusculotendonUnit::GetNumSegs() const
{
	return GetNumAttachPts() - 1;
}

void cMusculotendonUnit::SetupSegActuations(std::vector<tSegActuation>& out_actuations) const
{
	out_actuations.clear();
	int num_pts = GetNumAttachPts();
	if (num_pts > 1)
	{
		const auto& joint_mat = mChar->GetJointMat();

		int prev_joint = GetAttachPt(0).mJointID;
		for (int i = 1; i < num_pts; ++i)
		{
			const tAttachPt& pt = GetAttachPt(i);
			int curr_joint = pt.mJointID;

			tSegActuation curr_actuation;
			curr_actuation.mSegID = i - 1;

			assert(curr_joint <= curr_joint); // joints need to be ordered parent before child
			while (curr_joint > prev_joint)
			{
				curr_actuation.mJointIDs.push_back(curr_joint);
				curr_joint = cKinTree::GetParent(joint_mat, curr_joint);
			}

			if (curr_actuation.mJointIDs.size() > 0)
			{
				out_actuations.push_back(curr_actuation);
			}
			prev_joint = pt.mJointID;
		}
	}
}

double cMusculotendonUnit::GetRestLength() const
{
	double rest_len = mParams.mOptCELength + mParams.mSlackLength;
	return rest_len;
}

double cMusculotendonUnit::CalcForceSerial(double serial_len) const
{
	double strain = serial_len / mParams.mSlackLength - 1;
	double f_se = strain / gRefStrain;
	f_se = std::max(f_se, 0.0);
	f_se *= f_se;
	//f_se *= mParams.mForceMax; // mForceMax gets divided out later so skip mult
	return f_se;
}

double cMusculotendonUnit::CalcForceParallel(double parallel_len) const
{
	double norm_len = parallel_len / mParams.mOptCELength;
	double hpe = (norm_len - 1) / gOmega;
	double lpe = (1 - gOmega - norm_len) / (0.5 * gOmega);

	hpe = std::max(hpe, 0.0);
	lpe = std::max(lpe, 0.0);
	hpe *= hpe;
	lpe *= lpe;

	double f_pe = hpe - lpe;
	//f_pe *= mParams.mForceMax;
	return f_pe;
}

double cMusculotendonUnit::CalcForceElasticBuffer(double ce_len) const
{
	double norm_len = ce_len / mParams.mOptCELength;
	double f_be = (1 - gOmega - norm_len) / (0.5 * gOmega);
	f_be = std::max(f_be, 0.0);
	f_be *= f_be;
	//f_pe *= mParams.mForceMax;
	return f_be;
}

double cMusculotendonUnit::CalcForceVel(double vel_ce) const
{
	double norm_vel = vel_ce / mParams.mOptCELength;
	double fv = 0;
	if (norm_vel < 0)
	{
		fv = (gMaxVel - norm_vel) / (gMaxVel + gK * norm_vel);
	}
	else
	{
		fv = gN + (gN - 1) * (gMaxVel + norm_vel) / (7.56 * gK * norm_vel - gMaxVel);
	}
	fv = cMathUtil::Clamp(fv, gFvMin, gFvMax);
	return fv;
}

double cMusculotendonUnit::CalcForceLengthRel(double ce_len) const
{
	double fl = (ce_len / mParams.mOptCELength - 1) / gOmega;
	fl = std::abs(fl);
	fl = fl * fl * fl;
	fl *= std::log(gC);
	fl = std::exp(fl);
	return fl;
}

double cMusculotendonUnit::CalcForceParallelRel(double ce_len) const
{
	double f_pe = (ce_len / mParams.mOptCELength - 1) / gOmega;
	f_pe = std::max(0.0, f_pe);
	f_pe *= f_pe;
	return f_pe;
}

double cMusculotendonUnit::CalcVelCE(double fv) const
{
	fv = cMathUtil::Clamp(fv, gFvMin, gFvMax);

	double vel_ce = 0;
	if (fv < 1)
	{
		vel_ce = (fv - 1) / (-gK * fv - 1);
	}
	else
	{
		double a = (fv - gN) / (gN - 1);
		vel_ce = (a + 1) / (7.56 * gK * a - 1);
	}

	vel_ce *= gMaxVel;
	return vel_ce;
}

double cMusculotendonUnit::CalcLengthIntern() const
{
	double len = 0;
#if defined(ENABLE_ANALYTIC_ARM)
	len = GetOptCELength() + GetSlackLength();
	for (size_t i = 0; i < mSegActuations.size(); ++i)
	{
		const tSegActuation& actuation = mSegActuations[i];
		int seg_id = actuation.mSegID;
		const tAttachPt& attach_pt = GetAttachPt(seg_id + 1);

		for (size_t j = 0; j < actuation.mJointIDs.size(); ++j)
		{
			int joint_id = actuation.mJointIDs[j];
			const cJoint& joint = mChar->GetJoint(joint_id);

			double joint_theta;
			tVector joint_axis;
			joint.CalcRotation(joint_axis, joint_theta);

			double arm = attach_pt.mArm;
			bool fixed_arm = attach_pt.mFixedArm;
			double d_len = 0;

			if (fixed_arm)
			{
				d_len = joint_theta - attach_pt.mThetaRest;
			}
			else
			{
				d_len = sin(joint_theta - attach_pt.mThetaMax) - sin(attach_pt.mThetaRest - attach_pt.mThetaMax);
			}
			d_len *= mParams.mPennation * -arm;
			len += d_len;
		}
	}
#else
	len = CalcLenth();
#endif
	return len;
}

double cMusculotendonUnit::CalcSerialLen() const
{
	double len = CalcLengthIntern();
	double serial_len = len - mCELength;
	serial_len = std::max(gMinLen, serial_len);
	return serial_len;
}

void cMusculotendonUnit::UpdateActivation(double time_step)
{
	double exc = mExcitation;
#if defined(ENABLE_SOFT_EXCITATION_CLAMP)
	const double gamma = 4;
	const double bias = -0.5;
	exc = cMathUtil::Sigmoid(mExcitation, gamma, bias);
	exc = (gMaxActivation - gMinActivation) * exc + gMinActivation;
#endif // ENABLE_SOFT_EXCITATION_CLAMP

	double da = time_step * mActivationRate;
	mActivation = (1 - da) * mActivation + da * exc;

#if defined(ENABLE_ACTIVATION_CTRL)
	mActivation = exc;
#endif // ENABLE_ACTIVATION_CTRL

	mActivation = cMathUtil::Clamp(mActivation, gMinActivation, gMaxActivation);
}

void cMusculotendonUnit::ApplyForce(double force, Eigen::VectorXd& out_tau) const
{
	for (size_t i = 0; i < mSegActuations.size(); ++i)
	{
		const tSegActuation& curr_seg = mSegActuations[i];
		int seg_id = curr_seg.mSegID;
		
		tVector start_pos = CalcAttachPtWorldPos(seg_id);
		tVector end_pos = CalcAttachPtWorldPos(seg_id + 1);
		tVector mid_pos = 0.5 * (end_pos + start_pos);
		tVector dir = (end_pos - start_pos).normalized();
		
		const tAttachPt& attach_pt = GetAttachPt(seg_id + 1);
		bool fixed_arm = attach_pt.mFixedArm;

		for (size_t j = 0; j < curr_seg.mJointIDs.size(); ++j)
		{
			int joint_id = curr_seg.mJointIDs[j];
			tVector joint_pos = mChar->CalcJointPos(joint_id);
			tVector r = mid_pos - joint_pos;
			r = r.cross3(dir);

			cJoint& joint = mChar->GetJoint(joint_id);
			tVector axis = joint.CalcAxisWorld();
			double arm = axis.dot(r);

#if defined(ENABLE_FIXED_ARM)
			if (fixed_arm)
			{
				double fixed_arm_len = attach_pt.mArm;
				fixed_arm_len = std::abs(fixed_arm_len);
				arm = (arm < 0) ? -fixed_arm_len : fixed_arm_len;
			}
#endif // ENABLE_FIXED_ARM

#if defined(ENABLE_ANALYTIC_ARM)
			arm = attach_pt.mArm;
			if (!fixed_arm)
			{
				double joint_theta;
				tVector joint_axis;
				joint.CalcRotation(joint_axis, joint_theta);

				double d_theta = joint_theta - attach_pt.mThetaMax;
				arm *= std::max(0.0, std::cos(d_theta));
			}
			arm *= -1;
#endif
			double tau = force * arm;

			int param_offset = mChar->GetParamOffset(joint_id);
			int param_size = mChar->GetParamSize(joint_id);
			out_tau.segment(param_offset, param_size) += tau * Eigen::VectorXd::Ones(param_size);
		}
	}
}


void cMusculotendonUnit::PostProcessParams(Eigen::VectorXd& out_params) const
{
	for (int i = 0; i < eOptParamMax; ++i)
	{
		const tParamData& data = gOptParamData[i];
		double val_min = data.mValMin;
		double val_max = data.mValMax;

		double val = out_params[i];
		val = cMathUtil::Clamp(val, val_min, val_max);
		
		out_params[i] = val;
	}

	int num_active_segs = static_cast<int>(mSegActuations.size());
	for (int s = 0; s < num_active_segs; ++s)
	{
		int offset = eOptParamMax + s * eOptSegParamMax;
		for (int i = 0; i < eOptSegParamMax; ++i)
		{
			const tParamData& data = gOptSegParamData[i];
			double val_min = data.mValMin;
			double val_max = data.mValMax;

			double val = out_params[offset + i];

			if (i == eOptSegParamArm)
			{
				double val_sign = cMathUtil::Sign(val);
				val = std::abs(val);
				val = cMathUtil::Clamp(val, val_min, val_max);
				val *= val_sign;
			}
			else
			{
				val = cMathUtil::Clamp(val, val_min, val_max);
			}

			out_params[offset + i] = val;;
		}

		const tSegActuation& seg = mSegActuations[s];
		const tAttachPt& attach_pt = GetAttachPt(seg.mSegID + 1);
		if (attach_pt.mFixedArm)
		{
			out_params[offset + eOptSegParamThetaMax] = 0;
		}
	}
}