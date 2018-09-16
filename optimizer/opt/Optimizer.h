#pragma once

#include <memory>
#include <string>
#include <vector>

#include "OptFunc.h"

// tries to MINIMIZE a function as represented by a cOptFunc
class cOptimizer {
  public:
    struct tOptParams {
        std::string mOutputFile;
        std::string mIntOutputPath;
        bool mEnableOutputLog;

        tOptParams();
    };

    cOptimizer(void);
    virtual ~cOptimizer(void);

    virtual void Init(std::shared_ptr<cOptFunc> &opt_func);
    virtual cOptFunc::tPoint Optimize(const tOptParams &params) = 0;
    virtual void OutputIntermediateResult(bool enable);

  protected:
    std::shared_ptr<cOptFunc> mOptFunc;

    int mDimensions;
    cOptFunc::tPoint mPt;
    cOptFunc::tPoint mBestPt;

    // approximate scale of each dimension such that 1 / mScale will
    // normalize all dimensions to be roughly the same
    cOptFunc::tPoint mScale;

    double mBestObjVal;
    bool mEnableIntOutput;

    virtual void OutputPtInt(const cOptFunc::tPoint &pt, double obj_val, const std::string &filename_noext) const;
    virtual void OutputPt(const cOptFunc::tPoint &pt, const std::string &filename) const;
    virtual void OutputLog(const std::vector<double> &fit_vals, double final_val,
                           const std::string &filename_noext) const;
};
