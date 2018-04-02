#pragma once

#include <vector>
#include "opt/Optimizer.h"
#include <cmaes.h>

// tries to MINIMIZE a function as represented by a cOptFunc
class cOptCMA : public cOptimizer
{
public:
	struct tCMAParams
	{
	public:
		double mSigma;
		int mPopSize;
		double mStepTol;
		int mMaxGen;
		int mMaxIter;

		tCMAParams()
		{
			mSigma = 1.0;
			mPopSize = 0; // defaults to whatever the algorithm decides
			mStepTol = 0;
			mMaxGen = 10;
			mMaxIter = 1;
		}
	};

	struct tCMALog
	{
		int mNumGens;
		double mFinalStepSize;
		double mTime; // in hours
		double mInitVal;
		double mObjVal;

		tCMALog();
	};

	cOptCMA();
	virtual ~cOptCMA(void);

	virtual cOptFunc::tPoint Optimize(const tOptParams& params);

	virtual void SetCMAParams(const tCMAParams& params);
	virtual const tCMALog& GetLog() const;

protected:
	tCMAParams mParams;
	tCMALog mLog;

	CMAES<double> mCMA;

	virtual void InitCMA();
	virtual void InitPt(cOptFunc::tPoint& pt);

	virtual bool CheckTermination();
	virtual int GetCurrGen() const ;
	virtual double GetCurrSigma() const;
	virtual int GetEvalCount() const;
	virtual cOptFunc::tPoint GetCurrBestPt() const;

	virtual void EvalPop(double* const* pop, int pop_size, std::vector<double>& out_func_vals);
	virtual void EvalPt(const cOptFunc::tPoint& pt, double* out_val);

	virtual void OutputPt(cOptFunc::tPoint& pt, double val, const std::string& filename, const std::string& int_filename_noext) const;
};
