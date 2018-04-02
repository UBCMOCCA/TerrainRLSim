#include "CtMTUController.h"
#include "sim/SimCharacter.h"

//#define ENABLE_SERIAL_STATE

const std::string gMTUsKey = "MusculotendonUnits";

cCtMTUController::cCtMTUController() : cCtController()
{
}

cCtMTUController::~cCtMTUController()
{
}

void cCtMTUController::Init(cSimCharacter* character, const std::string& param_file)
{
	cCtController::Init(character);
	LoadParams(param_file);

	InitPoliState();
	InitCurrAction();
	SetupActionBounds();
}

void cCtMTUController::Reset()
{
	cCtController::Reset();
	ResetMTUs();
}

void cCtMTUController::Clear()
{
	cCtController::Clear();
	mMTUs.clear();
}

bool cCtMTUController::LoadParams(const std::string& param_file)
{
	bool succ = LoadMTUs(param_file);
	return succ;
}

int cCtMTUController::GetNumMTUs() const
{
	return static_cast<int>(mMTUs.size());
}

const cMusculotendonUnit& cCtMTUController::GetMTU(int id) const
{
	return mMTUs[id];
}

void cCtMTUController::HandlePoseReset()
{
	cCtController::HandlePoseReset();
	ResetCEState();
}

void cCtMTUController::HandleVelReset()
{
	cCtController::HandleVelReset();
	ResetCEState();
}

int cCtMTUController::GetPoliStateSize() const
{
	int state_size = cCtController::GetPoliStateSize();
	state_size += GetMTUStateSize();
	return state_size;
}

int cCtMTUController::GetPoliActionSize() const
{
	return GetNumMTUs();
}

void cCtMTUController::BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	double min_act = cMusculotendonUnit::gMinActivation;
	double max_act = cMusculotendonUnit::gMaxActivation;

	//const double activation_offset = 0;
	//const double activation_scale = 1;
	//const double activation_offset = -0.5;
	//const double activation_scale = 2;
	const double activation_offset = -0.5 * (max_act + min_act);
	const double activation_scale = 1.5 / (max_act - min_act);

	int output_size = GetPoliActionSize();
	out_offset = activation_offset * Eigen::VectorXd::Ones(output_size);
	out_scale = activation_scale * Eigen::VectorXd::Ones(output_size);
}

int cCtMTUController::GetNumOptParams() const
{
	int num_params = 0;
	for (int i = 0; i < GetNumMTUs(); ++i)
	{
		int curr_num_params = GetNumOptParams(i);
		num_params += curr_num_params;
	}
	return num_params;
}

int cCtMTUController::GetNumOptParams(int mtu_id) const
{
	const cMusculotendonUnit& curr_mtu = GetMTU(mtu_id);
	int num_params = curr_mtu.GetNumOptParams();
	return num_params;
}

void cCtMTUController::FetchOptParamScale(Eigen::VectorXd& out_scale) const
{
	int num_params = GetNumOptParams();
	out_scale.resize(num_params);

	int param_offset = 0;
	int param_size = 0;
	for (int i = 0; i < GetNumMTUs(); ++i)
	{
		Eigen::VectorXd curr_params;
		const cMusculotendonUnit& curr_mtu = mMTUs[i];
		curr_mtu.FetchOptParamScale(curr_params);

		param_size = static_cast<int>(curr_params.size());
		out_scale.segment(param_offset, param_size) = curr_params;
		param_offset += param_size;
	}

	assert(param_offset == num_params);
}

void cCtMTUController::BuildOptParams(Eigen::VectorXd& out_params) const
{
	int num_params = GetNumOptParams();
	out_params.resize(num_params);

	int param_offset = 0;
	int param_size = 0;
	for (int i = 0; i < GetNumMTUs(); ++i)
	{
		Eigen::VectorXd curr_params;
		const cMusculotendonUnit& curr_mtu = mMTUs[i];
		curr_mtu.BuildOptParams(curr_params);

		param_size = static_cast<int>(curr_params.size());
		out_params.segment(param_offset, param_size) = curr_params;
		param_offset += param_size;
	}

	assert(param_offset == num_params);
}

void cCtMTUController::SetOptParams(const Eigen::VectorXd& opt_params)
{
	int param_offset = 0;
	int param_size = 0;
	for (int i = 0; i < GetNumMTUs(); ++i)
	{
		cMusculotendonUnit& curr_mtu = mMTUs[i];
		param_size = GetNumOptParams(i);
		Eigen::VectorXd curr_params = opt_params.segment(param_offset, param_size);
		curr_mtu.SetOptParams(curr_params);

		param_offset += param_size;
	}

	int num_params = GetNumOptParams();
	assert(param_offset == num_params);
}

int cCtMTUController::CalcOptParamOffset(int mtu_id) const
{
	int param_offset = 0;
	int param_size = 0;
	for (int i = 0; i < mtu_id; ++i)
	{
		param_size = GetNumOptParams(i);
		param_offset += param_size;
	}
	return param_offset;
}

void cCtMTUController::OutputOptParams(FILE* f, const Eigen::VectorXd& params) const
{
	std::string json = BuildOptParamsJson(params);
	json = "{\n\"" + gMTUsKey + "\":\n" + json + "\n}";
	fprintf(f, "%s", json.c_str());
}

std::string cCtMTUController::BuildOptParamsJson(const Eigen::VectorXd& params) const
{
	std::string json = "[\n";

	int offset = 0;
	int size = 0;
	for (int i = 0; i < GetNumMTUs(); ++i)
	{
		const cMusculotendonUnit& mtu = GetMTU(i);
		size = GetNumOptParams(i);
		Eigen::VectorXd curr_params = params.segment(offset, size);
		std::string curr_json = mtu.BuildOptParamsJson(curr_params);

		if (i != 0)
		{
			json += ",\n";
		}
		json += curr_json;

		offset += size;
	}
	json += "\n]";

	int num_params = GetNumOptParams();
	assert(num_params == offset);

	return json;
}

bool cCtMTUController::LoadMTUs(const std::string& param_file)
{
	std::ifstream f_stream(param_file);
	Json::Reader reader;
	Json::Value root;
	bool succ = reader.parse(f_stream, root);
	f_stream.close();

	if (succ)
	{
		if (!root[gMTUsKey].isNull())
		{
			const Json::Value& mtus_arr = root[gMTUsKey];
			assert(mtus_arr.isArray());

			int num_mtus = mtus_arr.size();
			mMTUs.resize(num_mtus);

			for (int i = 0; i < num_mtus; ++i)
			{
				const Json::Value& mtu_json = mtus_arr.get(i, 0);
				cMusculotendonUnit::tParams params;
				bool param_succ = cMusculotendonUnit::ParseParams(mtu_json, params);

				if (param_succ)
				{
					cMusculotendonUnit& curr_mtu = mMTUs[i];
					curr_mtu.Init(mChar, params);
				}
				else
				{
					succ = false;
					break;
				}
			}
		}

		if (!succ)
		{
			mMTUs.clear();
			printf("Failed to parse MTUParams from %s\n", param_file.c_str());
			assert(false); // failed to parse MTU params
		}
	}

	return succ;
}

void cCtMTUController::ResetMTUs()
{
	int num_mtus = GetNumMTUs();
	for (int i = 0; i < num_mtus; ++i)
	{
		auto& mtu = mMTUs[i];
		mtu.Reset();
	}
}

void cCtMTUController::ResetCEState()
{
	for (int i = 0; i < GetNumMTUs(); ++i)
	{
		mMTUs[i].ResetCELength();
	}
}

void cCtMTUController::SetupActionBounds()
{
	//const double min_excitation = -std::numeric_limits<double>::infinity();
	//const double max_excitation = std::numeric_limits<double>::infinity();
	const double min_excitation = cMusculotendonUnit::gMinActivation;
	const double max_excitation = cMusculotendonUnit::gMaxActivation;

	int action_size = GetPoliActionSize();
	mActionBoundMin = min_excitation * Eigen::VectorXd::Ones(action_size);
	mActionBoundMax = max_excitation * Eigen::VectorXd::Ones(action_size);
}

void cCtMTUController::UpdateCalcTau(double time_step, Eigen::VectorXd& out_tau)
{
	if (NewActionUpdate() && !mFirstCycle)
	{
		mUpdateCounter = 0;
	}

	mUpdateCounter += time_step;

	if (NewActionUpdate())
	{
		UpdateAction();
		mFirstCycle = false;
	}
	
	UpdateMTUs(time_step, out_tau);
}

void cCtMTUController::UpdateMTUs(double time_step, Eigen::VectorXd& out_tau)
{
	int num_dof = mChar->GetNumDof();
	out_tau = Eigen::VectorXd::Zero(num_dof);

	int num_mtus = GetNumMTUs();
	for (int i = 0; i < num_mtus; ++i)
	{
		Eigen::VectorXd mtu_tau;
		mMTUs[i].UpdateCalcTau(time_step, mtu_tau);
		out_tau += mtu_tau;
	}
}

void cCtMTUController::ApplyAction(const tAction& action)
{
	cCtController::ApplyAction(action);

	int num_mtus = GetNumMTUs();
	for (int i = 0; i < num_mtus; ++i)
	{
		double u = mCurrAction.mParams[i];
		cMusculotendonUnit& mtu = mMTUs[i];
		mtu.SetExcitation(u);
	}
}

int cCtMTUController::GetMTUStateSize() const
{
	int mtu_size = GetNumMTUs();

#if defined(ENABLE_SERIAL_STATE)
	mtu_size += GetNumMTUs();
#endif // ENABLE_SERIAL_STATE

	return mtu_size;
}

int cCtMTUController::GetMTUStateOffset() const
{
	return cCtController::GetPoliStateSize();
}

void cCtMTUController::BuildPoliState(Eigen::VectorXd& out_state) const
{
	cCtController::BuildPoliState(out_state);

	Eigen::VectorXd mtu_state;
	BuildMTUState(mtu_state);

	int mtu_offset = GetMTUStateOffset();
	int mtu_size = GetMTUStateSize();
	out_state.segment(mtu_offset, mtu_size) = mtu_state;
}

void cCtMTUController::BuildMTUState(Eigen::VectorXd& out_state) const
{
	out_state.resize(GetMTUStateSize());
	int num_mtus = GetNumMTUs();
	for (int i = 0; i < num_mtus; ++i)
	{
		const auto& mtu = mMTUs[i];
		double ce = mtu.CalcCEStrain();
		out_state(i) = ce;

#if defined(ENABLE_SERIAL_STATE)
		double se = mtu.CalcSEStrain();
		double ref_strain = cMusculotendonUnit::GetRefStrain();
		se /= ref_strain;
		out_state(num_mtus + i) = se;
#endif // ENABLE_SERIAL_STATE
	}
}

void cCtMTUController::ApplyExpNoise(tAction& out_action)
{
	cCtController::ApplyExpNoise(out_action);
}
