#pragma once

#include "learning/AsyncMACETrainer.h"
#include "learning/MACETrainer.h"
#include "scenarios/ScenarioTrainCacla.h"

class cScenarioTrainMACE : public cScenarioTrain {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    cScenarioTrainMACE();
    virtual ~cScenarioTrainMACE();

    virtual std::string GetName() const;

  protected:
    virtual void BuildTrainer(std::shared_ptr<cTrainerInterface> &out_trainer);
    virtual void BuildExpScene(int id, std::shared_ptr<cScenarioExp> &out_exp) const;
    virtual void GetFragParams(int &out_num_frags, int &out_frag_size) const;
};
