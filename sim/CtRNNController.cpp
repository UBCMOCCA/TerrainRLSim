#include "CtRNNController.h"
#include "sim/SimCharacter.h"

cCtRNNController::cCtRNNController() : cCtController()
{
}

cCtRNNController::~cCtRNNController()
{
}

void cCtRNNController::BuildNet(std::unique_ptr<cNeuralNet>& out_net) const
{
	out_net = std::unique_ptr<cRecurrentNet>(new cRecurrentNet());
}

const cRecurrentNet* cCtRNNController::GetRNN() const
{
	const auto& net = GetNet();
	return static_cast<const cRecurrentNet*>(net.get());
}

void cCtRNNController::ExploitPolicy(const Eigen::VectorXd& state, tAction& out_action)
{
	const auto* rnn = GetRNN();
	bool is_start = mFirstCycle;
	Eigen::VectorXd params;
	rnn->Eval(state, is_start, params);
	assert(params.size() == GetPoliActionSize());

	out_action.mID = gInvalidIdx;
	out_action.mParams = params;

#if defined (ENABLE_DEBUG_PRINT)
	//DebugPrintAction(out_action);
	//printf("\n");
#endif
}