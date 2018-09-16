#include "CaclaTrainer.h"
#include "ACLearner.h"
#include "QNetTrainer.h"
#include "util/FileUtil.h"

//#define ENABLE_CRITIC_IS
//#define ENABLE_INVERTING_GRADIENTS
//#define OUTPUT_TD_TEST
#define CLAMP_VAL

#if defined(OUTPUT_TD_TEST)
const std::string gTDTestOutputFile = "output/td_test.txt";
#endif

cCaclaTrainer::cCaclaTrainer() { mHasActionBounds = false; }

cCaclaTrainer::~cCaclaTrainer() {}

void cCaclaTrainer::Init(const tParams &params) {
    cACTrainer::Init(params);

    mOffPolicyBuffer.clear();
    InitBatchBuffers();
    InitActionBounds();

    mActionCovar = Eigen::VectorXd::Ones(GetActionSize());
}

void cCaclaTrainer::Clear() {
    cACTrainer::Clear();

    mOffPolicyBuffer.clear();
    mActorBatchTDBuffer.clear();
    mBatchXBuffer.resize(0, 0);
    mBatchYBuffer.resize(0, 0);
    mBatchValBuffer0.resize(0);
    mBatchValBuffer1.resize(0);

    mActionMin.resize(0);
    mActionMax.resize(0);
    mActionCovar.resize(0);
}

void cCaclaTrainer::Reset() {
    cACTrainer::Reset();
    mOffPolicyBuffer.clear();
    mActorBatchTDBuffer.clear();
}

int cCaclaTrainer::AddTuple(const tExpTuple &tuple, int prev_id, int learner_id) {
    int t = cACTrainer::AddTuple(tuple, prev_id, learner_id);
    UpdateBuffers(t);
    return t;
}

void cCaclaTrainer::SetActionBounds(const Eigen::VectorXd &action_min, const Eigen::VectorXd &action_max) {
    assert(action_min.size() == action_max.size());
    mActionMin = action_min;
    mActionMax = action_max;

    mHasActionBounds = false;
    for (int i = 0; i < static_cast<int>(mActionMin.size()); ++i) {
        double curr_min = mActionMin[i];
        double curr_max = mActionMin[i];
        if (std::isfinite(curr_min) || std::isfinite(curr_max)) {
            mHasActionBounds = true;
            break;
        }
    }
}

void cCaclaTrainer::ResetCriticWeights() {
    cACTrainer::ResetCriticWeights();
    SyncTargetNets();
}

void cCaclaTrainer::SetActionCovar(const Eigen::VectorXd &action_covar) { mActionCovar = action_covar; }

void cCaclaTrainer::InitBatchBuffers() {
    int batch_size = GetBatchSize();
    if (batch_size > 0) {
        int input_size = GetInputSize();
        int output_size = GetOutputSize();
        mBatchXBuffer = Eigen::MatrixXd::Zero(batch_size, input_size);
        mBatchYBuffer = Eigen::MatrixXd::Zero(batch_size, output_size);
        mBatchValBuffer0 = Eigen::VectorXd::Zero(batch_size);
        mBatchValBuffer1 = Eigen::VectorXd::Zero(batch_size);
    }
    mActorBatchTDBuffer.clear();
}

void cCaclaTrainer::InitProblem(cNeuralNet::tProblem &out_prob) const {
    cACTrainer::InitProblem(out_prob);

#if defined(ENABLE_CRITIC_IS)
    const int output_size = GetCriticOutputSize();
    const int batch_size = GetBatchSize();
    out_prob.mW = Eigen::MatrixXd::Ones(batch_size, output_size);
#endif // ENABLE_CRITIC_IS
}

void cCaclaTrainer::InitActorProblem(cNeuralNet::tProblem &out_prob) const {
    const int input_size = GetActorInputSize();
    const int output_size = GetActorOutputSize();
    const int batch_size = GetBatchSize();

    out_prob.mX.resize(batch_size, input_size);
    out_prob.mY.resize(batch_size, output_size);
    out_prob.mW = Eigen::MatrixXd::Ones(batch_size, output_size);
    out_prob.mPassesPerStep = 1;
}

void cCaclaTrainer::InitActionBounds() {
    int action_size = GetActionSize();
    SetActionBounds(-std::numeric_limits<double>::infinity() * Eigen::VectorXd::Ones(action_size),
                    std::numeric_limits<double>::infinity() * Eigen::VectorXd::Ones(action_size));
}

void cCaclaTrainer::BuildNetPool(const std::string &net_file, const std::string &solver_file, int pool_size) {
    cACTrainer::BuildNetPool(net_file, solver_file, pool_size);
    SyncTargetNets();
}

void cCaclaTrainer::FetchActorMinibatch(int batch_size, std::vector<int> &out_batch) {
    int num_exp_tuples = static_cast<int>(mOffPolicyBuffer.size());
    int num_samples = std::min(batch_size, num_exp_tuples);
    out_batch.clear();
    out_batch.reserve(num_samples);

    for (int i = 0; i < num_samples; ++i) {
        bool contains = false;
        int t = gInvalidIdx;

        if (mParams.mPGEnableOnPolicy) {
            int idx = num_exp_tuples - i - 1;
            t = mOffPolicyBuffer[idx];
            contains = false;
        } else {
            int rand_idx = cMathUtil::RandInt(0, num_exp_tuples);
            t = mOffPolicyBuffer[rand_idx];

            contains = (std::find(mActorBatchBuffer.begin(), mActorBatchBuffer.end(), t) != mActorBatchBuffer.end()) ||
                       (std::find(out_batch.begin(), out_batch.end(), t) != out_batch.end());
        }

        if (!contains) {
            out_batch.push_back(t);
        }
    }

    if (mParams.mPGEnableOnPolicy) {
        mOffPolicyBuffer.erase(mOffPolicyBuffer.end() - num_samples, mOffPolicyBuffer.end());
    }
}

void cCaclaTrainer::BuildTupleY(int net_id, const tExpTuple &tuple, Eigen::VectorXd &out_y) {
    const auto &target_net = GetTargetNet(net_id);
    double new_v = CalcNewCumulativeReward(tuple, target_net);
    out_y = Eigen::VectorXd::Zero(GetOutputSize());
    out_y[0] = new_v;
}

void cCaclaTrainer::BuildTupleActorY(const tExpTuple &tuple, Eigen::VectorXd &out_y) {
    cACTrainer::BuildTupleActorY(tuple, out_y);
}

void cCaclaTrainer::Pretrain() {
    if (GetNumTuples() > 0) {
        for (int i = 0; i < mParams.mPretrainIters; ++i) {
            UpdateCritic();

            if (CheckUpdateTarget(i)) {
                UpdateTargetNet();
            }
        }
    }

    ResetSolvers();
    SyncTargetNets();
}

bool cCaclaTrainer::Step() {
    bool succ = cACTrainer::Step();

    int iter = GetIter();
    if (CheckUpdateTarget(iter)) {
        UpdateTargetNet();
    }

    return succ;
}

void cCaclaTrainer::BuildProblemY(int net_id, const std::vector<int> &tuple_ids, const Eigen::MatrixXd &X,
                                  cNeuralNet::tProblem &out_prob) {
    int num_data = static_cast<int>(tuple_ids.size());
    assert(num_data == GetBatchSize());
    assert(out_prob.mY.rows() == num_data);
    int target_net_id = GetTargetNetID(net_id);
    CalcNewCumulativeRewardBatch(target_net_id, tuple_ids, mBatchValBuffer0);

    out_prob.mY = mBatchValBuffer0;

#if defined(ENABLE_CRITIC_IS)
    int output_size = GetCriticOutputSize();
    if (mParams.mPGEnableImportanceSampling && !mParams.mPGEnableOnPolicy) {
        auto &actor_net = GetActor();
        Eigen::MatrixXd curr_actions;
        actor_net->EvalBatch(X, curr_actions);
        int batch_size = static_cast<int>(tuple_ids.size());

        for (int i = 0; i < batch_size; ++i) {
            int t = tuple_ids[i];
            const tExpTuple &tuple = GetTuple(t);

            Eigen::VectorXd curr_action = curr_actions.row(i);
            Eigen::VectorXd new_action;
            BuildTupleActorY(tuple, new_action);

            double logp = Calclogp(curr_action, new_action);
            double iw = std::exp(logp - tuple.mActionLogp);
            iw = std::min(iw, mParams.mPGIWClip);

            out_prob.mW.row(i) = iw * Eigen::VectorXd::Ones(output_size);
        }
    }
#endif // ENABLE_CRITIC_IS
}

int cCaclaTrainer::GetTargetNetID(int net_id) const {
    int target_id = net_id;
    int pool_size = GetNetPoolSize();
    if (EnableTargetNet()) {
        target_id = mParams.mPoolSize + net_id;
    } else if (pool_size > 1) {
        target_id = cMathUtil::RandIntExclude(0, pool_size, net_id);
    }

    return target_id;
}

const std::unique_ptr<cNeuralNet> &cCaclaTrainer::GetCriticTarget() const { return GetTargetNet(mCurrActiveNet); }

double cCaclaTrainer::CalcCurrCumulativeReward(const tExpTuple &tuple, const std::unique_ptr<cNeuralNet> &net) {
    Eigen::VectorXd x;
    BuildTupleX(tuple, x);

    Eigen::VectorXd y_next;
    net->Eval(x, y_next);
    double val = y_next[0];

    return val;
}

double cCaclaTrainer::CalcNewCumulativeReward(const tExpTuple &tuple, const std::unique_ptr<cNeuralNet> &net) {
    double val = 0;
    double r = tuple.mReward;

    double discount = GetDiscount();
    double norm_r = NormalizeReward(r);

    bool fail = tuple.GetFlag(tExpTuple::eFlagFail);
    if (fail) {
        val = norm_r;
    } else {
        Eigen::VectorXd x_next;
        BuildCriticXNext(tuple, x_next);

        Eigen::VectorXd y_next;
        net->Eval(x_next, y_next);

        double v_end = y_next[0];
#if defined(CLAMP_VAL)
        v_end = cMathUtil::Clamp(v_end, cQNetTrainer::gValClampMin, cQNetTrainer::gValClampMax);
#endif
        val = norm_r + discount * v_end;
    }

    return val;
}

void cCaclaTrainer::CalcCurrCumulativeRewardBatch(int net_id, const std::vector<int> &tuple_ids,
                                                  Eigen::VectorXd &out_vals) {
    const int num_data = static_cast<int>(tuple_ids.size());
    assert(num_data <= GetBatchSize());
    const auto &net = mNetPool[net_id];

    for (int i = 0; i < num_data; ++i) {
        int t = tuple_ids[i];
        tExpTuple tuple = GetTuple(t);
        Eigen::VectorXd x;
        BuildTupleX(tuple, x);
        mBatchXBuffer.row(i) = x;
    }

    net->EvalBatch(mBatchXBuffer, mBatchYBuffer);
    out_vals = mBatchYBuffer;
}

/*
void cCaclaTrainer::CalcNewCumulativeRewardBatchOld(int net_id, const std::vector<int>& tuple_ids,
                                                                                                Eigen::VectorXd&
out_vals)
{
        const int num_data = static_cast<int>(tuple_ids.size());
        assert(num_data <= GetBatchSize());
        const auto& net = mNetPool[net_id];

        for (int i = 0; i < num_data; ++i)
        {
                int t = tuple_ids[i];
                tExpTuple tuple = GetTuple(t);

                Eigen::VectorXd x_next;
                BuildCriticXNext(tuple, x_next);
                mBatchXBuffer.row(i) = x_next;
        }

        net->EvalBatch(mBatchXBuffer, mBatchYBuffer);
        PostProcessCumulativeReward(tuple_ids, mBatchYBuffer);

        double discount = GetDiscount();

        for (int i = 0; i < num_data; ++i)
        {
                int t = tuple_ids[i];
                tExpTuple tuple = GetTuple(t);

                double val = 0;
                double r = tuple.mReward;
                double norm_r = NormalizeReward(r);

                bool fail = tuple.GetFlag(tExpTuple::eFlagFail);
                if (fail)
                {
                        val = norm_r;
                }
                else
                {
                        double v_end = mBatchYBuffer(i, 0);
                        val = norm_r + discount * v_end;
                }

                out_vals(i) = val;
        }
}
*/

void cCaclaTrainer::CalcNewCumulativeRewardBatch(int net_id, const std::vector<int> &tuple_ids,
                                                 Eigen::VectorXd &out_vals) {
    const auto &net = mNetPool[net_id];
    const int num_data = static_cast<int>(tuple_ids.size());
    const int batch_size = GetBatchSize();
    assert(num_data <= batch_size);
    assert(mParams.mNumRewardSteps > 0);

    double discount = GetDiscount();
    double lambda = mParams.mTDLambda;

    std::vector<int> curr_ids = tuple_ids;
    Eigen::VectorXd discounts = Eigen::VectorXd::Ones(batch_size);
    Eigen::VectorXd cum_rewards = Eigen::VectorXd::Zero(batch_size);
    out_vals.setZero();

    for (int k = 0; k < mParams.mNumRewardSteps; ++k) {
        bool last_iter = (k == mParams.mNumRewardSteps - 1);

        for (int i = 0; i < num_data; ++i) {
            int t = curr_ids[i];
            if (t != gInvalidIdx) {
                tExpTuple tuple = GetTuple(t);
                double curr_discount = discounts[i];
                double r = tuple.mReward;
                double norm_r = NormalizeReward(r);

                cum_rewards[i] += curr_discount * norm_r;

                int next_t = gInvalidIdx;
                bool fail = tuple.GetFlag(tExpTuple::eFlagFail);
                if (fail) {
                    discounts[i] = 0;
                } else {
                    next_t = mExpBuffer->GetNextTupleID(t);
                    if (mParams.mEnableTDLambda || last_iter || next_t == gInvalidIdx) {
                        Eigen::VectorXd x_next;
                        BuildCriticXNext(tuple, x_next);
                        mBatchXBuffer.row(i) = x_next;
                    }
                    discounts[i] *= discount;
                }

                curr_ids[i] = next_t;
            }
        }

        if (last_iter || mParams.mEnableTDLambda) {
            net->EvalBatch(mBatchXBuffer, mBatchYBuffer);
            PostProcessCumulativeReward(curr_ids, mBatchYBuffer);

            for (int i = 0; i < num_data; ++i) {
                double val_scale = 1;

                int next_t = curr_ids[i];
                bool is_end = (next_t == gInvalidIdx) || last_iter;
                if (mParams.mEnableTDLambda) {
                    val_scale = std::pow(lambda, k);
                    if (!is_end) {
                        val_scale *= (1 - lambda);
                    }
                }

                double curr_val = cum_rewards[i] + discounts[i] * mBatchYBuffer(i, 0);
                curr_val *= val_scale;
                out_vals[i] += curr_val;

                if (mParams.mEnableTDLambda && is_end) {
                    discounts[i] = 0;
                    cum_rewards[i] = 0;
                }
            }
        }
    }
}

void cCaclaTrainer::PostProcessCumulativeReward(const std::vector<int> &tuple_ids, Eigen::MatrixXd &vals) const {
    // tuple_ids just for debugging
#if defined(CLAMP_VAL)
    for (int i = 0; i < vals.rows(); ++i) {
        for (int j = 0; j < vals.cols(); ++j) {
            double curr_val = vals(i, j);
            vals(i, j) = cMathUtil::Clamp(curr_val, cQNetTrainer::gValClampMin, cQNetTrainer::gValClampMax);
        }
    }
#endif
}

void cCaclaTrainer::BuildActorProblemY(const std::vector<int> &tuple_ids, const Eigen::MatrixXd &X,
                                       cNeuralNet::tProblem &out_prob) {
    ePGMode mode = GetPGMode();

    switch (mode) {
    case ePGModeCacla:
        BuildActorProblemYCacla(tuple_ids, X, out_prob);
        break;
    case ePGModeTD:
    case ePGModePTD:
        BuildActorProblemYTD(tuple_ids, X, out_prob);
        break;
    default:
        assert(false); // unsupported mode
        break;
    }
}

void cCaclaTrainer::BuildActorProblemYCacla(const std::vector<int> &tuple_ids, const Eigen::MatrixXd &X,
                                            cNeuralNet::tProblem &out_prob) {
    if (mHasActionBounds || mParams.mPGEnableImportanceSampling && !mParams.mPGEnableOnPolicy) {
        BuildActorProblemYTD(tuple_ids, X, out_prob);
    } else {
        cACTrainer::BuildActorProblemY(tuple_ids, X, out_prob);
        /*
        if (!mHasActionBounds)
        {
                cACTrainer::BuildActorProblemY(tuple_ids, X, out_prob);
        }
        else
        {
                auto& actor_net = GetActor();
                actor_net->EvalBatch(X, out_prob.mY);

                int batch_size = GetActorBatchSize();
                batch_size = std::min(batch_size, static_cast<int>(tuple_ids.size()));
                for (int i = 0; i < batch_size; ++i)
                {
                        int t = tuple_ids[i];
                        tExpTuple tuple = GetTuple(t);

                        Eigen::VectorXd curr_action = out_prob.mY.row(i);
                        Eigen::VectorXd new_action;
                        BuildTupleActorY(tuple, new_action);

                        const double adv = 1;
                        Eigen::VectorXd diff = new_action - curr_action;
                        ProcessPoliGrad(curr_action, adv, diff);
                        new_action = curr_action + diff;

                        out_prob.mY.row(i) = new_action;
                }
        }
        */
    }
}

void cCaclaTrainer::BuildActorProblemYTD(const std::vector<int> &tuple_ids, const Eigen::MatrixXd &X,
                                         cNeuralNet::tProblem &out_prob) {
    auto &actor_net = GetActor();
    actor_net->EvalBatch(X, out_prob.mY);

    int num_tuples = static_cast<int>(tuple_ids.size());
    int batch_size = GetActorBatchSize();
    batch_size = std::min(batch_size, num_tuples);

    // out_prob.mW.setZero();

    int action_size = GetActorOutputSize();
    for (int i = 0; i < batch_size; ++i) {
        int t = tuple_ids[i];
        const tExpTuple &tuple = GetTuple(t);

        Eigen::VectorXd curr_action = out_prob.mY.row(i);
        Eigen::VectorXd new_action;
        BuildTupleActorY(tuple, new_action);

        double td = mActorBatchTDBuffer[i];
        double adv = CalcAdvantage(td);

        if (mParams.mPGEnableImportanceSampling && !mParams.mPGEnableOnPolicy) {
            double logp = CalcLogp(curr_action, new_action);
            double iw = std::exp(logp - tuple.mActionLogp);
            iw = std::min(iw, mParams.mPGIWClip);

            adv *= iw;
        }

        adv = cMathUtil::Clamp(adv, -mParams.mPGAdvClip, mParams.mPGAdvClip);

        Eigen::VectorXd diff = new_action - curr_action;
        // ProcessPoliGrad(curr_action, adv, diff); // hack hack hack
        new_action = curr_action + diff;

        out_prob.mY.row(i) = new_action;
        out_prob.mW.row(i) = adv * Eigen::VectorXd::Ones(action_size);
    }
}

int cCaclaTrainer::GetPoolSize() const {
    int pool_size = GetNetPoolSize();
    if (EnableTargetNet()) {
        pool_size *= 2;
    }
    return pool_size;
}

void cCaclaTrainer::UpdateActorBatchBuffer() {
    int batch_size = GetActorBatchSize();
    FetchActorMinibatch(batch_size, mBatchBuffer);
    int num_samples = static_cast<int>(mBatchBuffer.size());
    Eigen::VectorXd td_buffer = Eigen::VectorXd::Zero(num_samples);

    int net_pool_size = GetNetPoolSize();
    for (int k = 0; k < net_pool_size; ++k) {
        int net_id = k;
        int target_net_id = net_id;
        if (EnableTargetNet()) {
            target_net_id = GetTargetNetID(net_id);
        }

        CalcCurrCumulativeRewardBatch(target_net_id, mBatchBuffer, mBatchValBuffer0);
        CalcNewCumulativeRewardBatch(target_net_id, mBatchBuffer, mBatchValBuffer1);

        for (int i = 0; i < num_samples; ++i) {
            double curr_val = mBatchValBuffer0[i];
            double new_val = mBatchValBuffer1[i];
            double td = new_val - curr_val;
            td_buffer[i] += td;
        }
    }
    td_buffer /= net_pool_size;

    for (int i = 0; i < num_samples; ++i) {
        double td = td_buffer[i];
        bool valid_td = CheckActorTD(td);
        if (valid_td) {
            int t = mBatchBuffer[i];
            mActorBatchBuffer.push_back(t);
            mActorBatchTDBuffer.push_back(td);
        }

#if defined(OUTPUT_TD_TEST)
        std::string td_str = std::to_string(td) + "\n";
        cFileUtil::AppendText(td_str, gTDTestOutputFile);
#endif
    }
}

void cCaclaTrainer::UpdateActorBatchBufferPostStep(int batch_size) {
    cACTrainer::UpdateActorBatchBufferPostStep(batch_size);
    mActorBatchTDBuffer.erase(mActorBatchTDBuffer.begin(), mActorBatchTDBuffer.begin() + batch_size);
}

bool cCaclaTrainer::EnableTargetNet() const {
    bool enable = (mParams.mFreezeTargetIters > 0);
    return enable;
}

bool cCaclaTrainer::CheckUpdateTarget(int iter) const {
    return EnableTargetNet() && (iter > 0) && (iter % mParams.mFreezeTargetIters == 0);
}

void cCaclaTrainer::UpdateTargetNet() { SyncTargetNets(); }

void cCaclaTrainer::SyncTargetNets() {
    if (EnableTargetNet()) {
        for (int i = 0; i < GetNetPoolSize(); ++i) {
            auto &net = mNetPool[i];
            auto &target_net = GetTargetNet(i);
            target_net->CopyModel(*net.get());
        }
    }
}

void cCaclaTrainer::UpdateBuffers(int t) {
    if (t != gInvalidIdx) {
        bool is_off_policy = IsOffPolicy(t);

        auto off_beg = mOffPolicyBuffer.begin();
        auto off_end = mOffPolicyBuffer.end();
        auto off_iter = std::find(off_beg, off_end, t);

        bool off_contains = off_iter != off_end;
        if (is_off_policy) {
            if (!off_contains) {
                mOffPolicyBuffer.push_back(t);
            }
        } else if (off_contains) {
            int idx = static_cast<int>(off_iter - off_beg);
            int last_val = mOffPolicyBuffer[mOffPolicyBuffer.size() - 1];
            mOffPolicyBuffer[idx] = last_val;
            mOffPolicyBuffer.pop_back();
        }

        auto actor_buffer_iter = std::find(mActorBatchBuffer.begin(), mActorBatchBuffer.end(), t);
        while (actor_buffer_iter != mActorBatchBuffer.end()) {
            int idx = static_cast<int>(actor_buffer_iter - mActorBatchBuffer.begin());
            int last_id = mActorBatchBuffer[mActorBatchBuffer.size() - 1];
            double last_td = mActorBatchTDBuffer[mActorBatchTDBuffer.size() - 1];

            mActorBatchBuffer[idx] = last_id;
            mActorBatchTDBuffer[idx] = last_td;
            mActorBatchBuffer.pop_back();
            mActorBatchTDBuffer.pop_back();

            actor_buffer_iter = std::find(mActorBatchBuffer.begin(), mActorBatchBuffer.end(), t);
        }
    }
    assert(mActorBatchBuffer.size() == mActorBatchTDBuffer.size());
}

bool cCaclaTrainer::IsOffPolicy(int t) const {
    int flag = mExpBuffer->GetFlags(t);
    bool off_policy = tExpTuple::TestFlag(flag, tExpTuple::eFlagOffPolicy);
    return off_policy;
}

cCaclaTrainer::ePGMode cCaclaTrainer::GetPGMode() const { return mParams.mPGMode; }

bool cCaclaTrainer::CheckActorTD(double td) const {
    bool valid = true;
    ePGMode mode = GetPGMode();
    if (mode == ePGModeCacla || mode == ePGModePTD) {
        if (td <= 0) {
            valid = false;
        }
    }
    return valid;
}

void cCaclaTrainer::ProcessPoliGrad(const Eigen::VectorXd &action, double scale, Eigen::VectorXd &out_diff) const {
#if defined(ENABLE_INVERTING_GRADIENTS)
    int action_size = static_cast<int>(action.size());
    assert(action.size() == action.size());
    assert(action.size() == mActionMin.size());
    assert(action.size() == mActionMax.size());

    if (scale != 0) {
        for (int i = 0; i < action_size; ++i) {
            double diff_val = cMathUtil::Sign(scale) * out_diff[i];
            double action_val = action[i];

            double bound_min = mActionMin[i];
            double bound_max = mActionMax[i];

            double diff_scale = 1;
            if (diff_val > 0) {
                diff_scale = (bound_max - action_val) / (bound_max - bound_min);
            } else {
                diff_scale = (action_val - bound_min) / (bound_max - bound_min);
            }

            diff_val *= diff_scale;
            diff_val *= cMathUtil::Sign(scale);
            out_diff[i] = diff_val;
        }
    }
#else
    int action_size = static_cast<int>(action.size());
    assert(action.size() == action.size());
    assert(action.size() == mActionMin.size());
    assert(action.size() == mActionMax.size());

    if (scale != 0) {
        for (int i = 0; i < action_size; ++i) {
            double diff_val = cMathUtil::Sign(scale) * out_diff[i];
            double action_val = action[i];

            double bound_min = mActionMin[i];
            double bound_max = mActionMax[i];

            if (std::isfinite(bound_max) && diff_val > 0 && action_val > bound_max) {
                diff_val = bound_max - action_val;
            } else if (std::isfinite(bound_min) && diff_val < 0 && action_val < bound_min) {
                diff_val = bound_min - action_val;
            }

            diff_val *= cMathUtil::Sign(scale);
            out_diff[i] = diff_val;
        }
    }
#endif // ENABLE_INVERTING_GRADIENTS
}

double cCaclaTrainer::CalcAdvantage(double td) const {
    double adv = 0;
    double adv_scale = mParams.mPGAdvScale;

    ePGMode mode = GetPGMode();
    switch (mode) {
    case ePGModeCacla:
        adv = (td > 0) ? 1 : 0;
        break;
    case ePGModeTD:
        adv = adv_scale * td;
        break;
    case ePGModePTD:
        adv = adv_scale * std::max(0.0, td);
        break;
    default:
        assert(false); // unsupported mode
        break;
    }

    return adv;
}

double cCaclaTrainer::CalcLogp(const Eigen::VectorXd &mean_action, const Eigen::VectorXd &sample_action) const {
    double logp = cMathUtil::EvalGaussianLogp(mean_action, mActionCovar, sample_action);
    return logp;
}
