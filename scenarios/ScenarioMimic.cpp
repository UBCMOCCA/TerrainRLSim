#include "ScenarioMimic.h"
#include "ScenarioExpMimic.h"
#include "sim/MixController.h"

//#define ENABLE_COACH_BLEND
//#define ENABLE_COACH_ACTIVE_PROB

cScenarioMimic::cScenarioMimic()
{
	mTrainerParams.mPoolSize = 1;
	mCoachActiveProb = 0;
	mInitCoachActiveProb = 1;
}

cScenarioMimic::~cScenarioMimic()
{
}

void cScenarioMimic::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioTrain::ParseArgs(parser);
	parser->ParseDouble("coach_active_prob", mCoachActiveProb);
	parser->ParseDouble("init_coach_active_prob", mInitCoachActiveProb);
}

void cScenarioMimic::EnableTraining(bool enable)
{
	cScenarioTrain::EnableTraining(enable);

	int iters = GetIter();
	double coach_blend = (mEnableTraining) ? CalcCoachBlend(iters) : 0;
	double coach_active_prob = (mEnableTraining) ? CalcCoachActiveProb(iters) : 0;
	for (size_t i = 0; i < mExpPool.size(); ++i)
	{
		auto mimc_scene = std::dynamic_pointer_cast<cScenarioExpMimic>(mExpPool[i]);
#if defined(ENABLE_COACH_BLEND)
		mimc_scene->SetCoachBlend(coach_blend);
#endif

#if defined(ENABLE_COACH_ACTIVE_PROB)
		mimc_scene->SetCoachActiveProb(coach_active_prob);
#endif
	}
}

std::string cScenarioMimic::GetName() const
{
	return "Mimic";
}

void cScenarioMimic::BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const
{
	out_exp = std::shared_ptr<cScenarioExpMimic>(new cScenarioExpMimic());
}

void cScenarioMimic::BuildTrainer(std::shared_ptr<cTrainerInterface>& out_trainer)
{
	if (mEnableAsyncMode)
	{
		auto trainer = std::shared_ptr<cAsyncTrainer>(new cAsyncTrainer());
		out_trainer = trainer;
	}
	else
	{
		auto trainer = std::shared_ptr<cNeuralNetTrainer>(new cNeuralNetTrainer());
		out_trainer = trainer;
	}
}

void cScenarioMimic::ResetScenePool()
{
	cScenarioTrain::ResetScenePool();
	for (int i = 0; i < GetPoolSize(); ++i)
	{
		auto scene = std::dynamic_pointer_cast<cScenarioExpMimic>(mExpPool[i]);
		scene->EnableCoachCtrl(true);

#if defined(ENABLE_COACH_ACTIVE_PROB)
		scene->EnableCoachActiveProb(true);
#endif
	}
}

int cScenarioMimic::GetCharCtrlID() const
{
	auto scene = std::dynamic_pointer_cast<cScenarioExpMimic>(mExpPool[0]);
	return scene->GetCharCtrlID();
}

int cScenarioMimic::GetCoachCtrlID() const
{
	auto scene = std::dynamic_pointer_cast<cScenarioExpMimic>(mExpPool[0]);
	return scene->GetCoachCtrlID();
}

void cScenarioMimic::UpdateExpScene(double time_step, int exp_id, std::shared_ptr<cScenarioExp>& out_exp, bool& out_done)
{
	auto& learner = mLearners[exp_id];

	cScenarioTrain::UpdateExpScene(time_step, exp_id, out_exp, out_done);

#if !defined(ENABLE_COACH_BLEND) && !defined(ENABLE_COACH_ACTIVE_PROB)
	// hack
	auto stage = learner->GetStage();
	if (stage != cNeuralNetTrainer::eStageInit)
	{
		auto scene = std::dynamic_pointer_cast<cScenarioExpMimic>(mExpPool[exp_id]);
		scene->EnableCoachCtrl(false);
	}
#endif
}

void cScenarioMimic::SetupLearner(const std::shared_ptr<cSimCharacter>& character, std::shared_ptr<cNeuralNetLearner>& out_learner) const
{
	int char_ctrl_id = GetCharCtrlID();
	std::shared_ptr<cMixController> mix_ctrl = std::static_pointer_cast<cMixController>(character->GetController());
	std::shared_ptr<cNNController> ctrl = std::static_pointer_cast<cNNController>(mix_ctrl->GetController(char_ctrl_id));

	auto& net = ctrl->GetNet();
	out_learner->SetNet(net.get());
}

void cScenarioMimic::SetupTrainerOffsetScale(const std::shared_ptr<cTrainerInterface>& out_trainer)
{
	bool valid_init_model = out_trainer->HasInitModel();
	if (!valid_init_model)
	{
		int char_ctrl_id = GetCharCtrlID();
		std::shared_ptr<cMixController> mix_ctrl = std::static_pointer_cast<cMixController>(cScenarioTrain::GetDefaultController());
		std::shared_ptr<cNNController> ctrl = std::static_pointer_cast<cNNController>(mix_ctrl->GetController(char_ctrl_id));

		Eigen::VectorXd input_offset;
		Eigen::VectorXd input_scale;
		ctrl->BuildNNInputOffsetScale(input_offset, input_scale);
		out_trainer->SetInputOffsetScale(input_offset, input_scale);

		Eigen::VectorXd output_offset;
		Eigen::VectorXd output_scale;
		ctrl->BuildNNOutputOffsetScale(output_offset, output_scale);
		out_trainer->SetOutputOffsetScale(output_offset, output_scale);
	}
}

void cScenarioMimic::UpdateExpSceneRates(int exp_id, std::shared_ptr<cScenarioExp>& out_exp) const
{
	cScenarioTrain::UpdateExpSceneRates(exp_id, out_exp);

	auto& learner = mLearners[exp_id];
	int iters = learner->GetIter();
	auto mimic_exp = std::dynamic_pointer_cast<cScenarioExpMimic>(out_exp);

#if defined(ENABLE_COACH_BLEND)
	double coach_blend = CalcCoachBlend(iters);
	mimic_exp->SetCoachBlend(coach_blend);
#endif

#if defined(ENABLE_COACH_ACTIVE_PROB)
	double coach_active_prob = CalcCoachActiveProb(iters);
	mimic_exp->SetCoachActiveProb(coach_active_prob);
#endif
}

double cScenarioMimic::CalcCoachBlend(int iters) const
{
	double lerp = static_cast<double>(iters) / mNumAnnealIters;
	lerp = cMathUtil::Clamp(lerp, 0.0, 1.0);
	double blend = 1 - lerp;
	return blend;
}

void cScenarioMimic::PrintLearnerInfo(int exp_id) const
{
	cScenarioTrain::PrintLearnerInfo(exp_id);

	auto& learner = mLearners[exp_id];
	int iters = learner->GetIter();

#if defined(ENABLE_COACH_BLEND)
	double coach_blend = CalcCoachBlend(iters);
	printf("Mimic Coach Blend: %.5f\n", coach_blend);
#endif

#if defined(ENABLE_COACH_ACTIVE_PROB)
	double coach_active_prob = CalcCoachActiveProb(iters);
	printf("Mimic Coach Active Prob: %.5f\n", coach_active_prob);
#endif
}

double cScenarioMimic::CalcCoachActiveProb(int iters) const
{
	double lerp = static_cast<double>(iters) / mNumAnnealIters;
	lerp = cMathUtil::Clamp(lerp, 0.0, 1.0);
	double prob = (1 - lerp) * mInitCoachActiveProb + lerp * mCoachActiveProb;
	return prob;
}