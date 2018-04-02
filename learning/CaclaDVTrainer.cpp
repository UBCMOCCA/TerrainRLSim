/*
 * CaclaDVTrainer.cpp
 *
 *  Created on: Oct 25, 2016
 *      Author: Glen
 */

#include "CaclaDVTrainer.h"

cCaclaDVTrainer::cCaclaDVTrainer() : cCaclaTrainer() {
	// TODO Auto-generated constructor stub

}

cCaclaDVTrainer::~cCaclaDVTrainer() {
	// TODO Auto-generated destructor stub
}

void cCaclaDVTrainer::UpdateCritic()
{
	int i = cMathUtil::RandInt(0, GetNetPoolSize());
	printf("Update DV Net %i:\n", i);
	bool succ = BuildProblem(i, mProb);
	if (succ)
	{
		UpdateNet(i, mProb);
	}
}

int cCaclaDVTrainer::GetTargetNetID(int net_id) const
{
	int target_id = net_id;
	int pool_size = GetNetPoolSize();
	if (EnableTargetNet())
	{
		target_id = mParams.mPoolSize + net_id;
	}
	else if (pool_size > 1)
	{
		target_id = cMathUtil::RandIntExclude(0, pool_size, net_id);
	}

	return target_id;
}


void cCaclaDVTrainer::BuildProblemY(int net_id, const std::vector<int>& tuple_ids,
	const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob)
{
	int num_data = static_cast<int>(tuple_ids.size());
	assert(num_data == GetBatchSize());
	assert(out_prob.mY.rows() == num_data);
	int target_net_id = GetTargetNetID(net_id);
	CalcNewCumulativeRewardBatch(target_net_id, tuple_ids, mBatchValBuffer0);
	out_prob.mY = mBatchValBuffer0;

#if defined(ENABLE_CRITIC_IS)
	int output_size = GetCriticOutputSize();
	if (mParams.mPGEnableImportanceSampling && !mParams.mPGEnableOnPolicy)
	{
		auto& actor_net = GetActor();
		Eigen::MatrixXd curr_actions;
		actor_net->EvalBatch(X, curr_actions);
		int batch_size = static_cast<int>(tuple_ids.size());

		for (int i = 0; i < batch_size; ++i)
		{
			int t = tuple_ids[i];
			const tExpTuple& tuple = GetTuple(t);

			Eigen::VectorXd curr_action = curr_actions.row(i);
			Eigen::VectorXd new_action;
			BuildTupleActorY(tuple, new_action);

			assert(tuple.mActionLogp > 0);
			double logp = CalcLogp(curr_action, new_action);
			double iw = std::exp(logp - tuple.mActionLogp);
			iw = std::min(iw, mParams.mPGIWClip);

			out_prob.mW.row(i) = iw * Eigen::VectorXd::Ones(output_size);
		}
	}
#endif // ENABLE_CRITIC_IS
}
