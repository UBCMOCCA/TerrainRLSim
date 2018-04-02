#pragma once

#include "scenarios/OptScenarioSimChar.h"

class cOptScenarioMTU : public  cOptScenarioSimChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cOptScenarioMTU(cOptimizer& optimizer);
	virtual ~cOptScenarioMTU();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Init();

	virtual int GetDimensions() const;
	virtual void InitPt(tPoint& pt) const;
	virtual void InitScale(tPoint& scale) const;

	virtual void SetModelFile(const std::string& model_file);
	virtual void SetCriticModelFile(const std::string& model_file);
	
	virtual bool IsThreadSafe() const;
	virtual void OutputPt(FILE*, const tPoint& pt) const;
	
	virtual std::string GetName() const;

protected:
	struct tMirrorParam
	{
		int mSrcID;
		int mDstID;
		int mSrcOffset;
		int mDstOffset;
	};

	int mMaxEpisodes;
	std::vector<int> mTargetMTUIDs;
	std::vector<int> mMTUParamOffsets;
	std::vector<tMirrorParam> mMirrorParams;

	int mNumOptParams;
	Eigen::VectorXd mDefaultFullParams;

	std::string mModelFile;
	std::string mCriticModelFile;

	virtual void BuildScene(int id, std::shared_ptr<cScenarioSimChar>& out_scene);
	virtual void InitScenePool();
	virtual void SetupTargetMTUs();
	virtual int GetNumTargetMTUs() const;
	virtual void ParseMirrorParams(const std::shared_ptr<cArgParser>& parser, std::vector<tMirrorParam>& out_params) const;

	virtual double CalcObjVal(const std::shared_ptr<cScenarioSimChar>& scene);
	virtual const std::shared_ptr<cCharController>& GetDefaultController() const;
	
	virtual double EvalPt(const tPoint& pt, const std::shared_ptr<cScenarioSimChar>& scene);
	virtual void SetOptParams(const tPoint& pt, const std::shared_ptr<cScenarioSimChar>& out_scene) const;
	virtual bool CheckTermination(const std::shared_ptr<cScenarioSimChar>& scene) const;

	virtual void FetchTargetOptParams(const Eigen::VectorXd& full_params, Eigen::VectorXd& out_params) const;
	virtual void ApplyTargetOptParams(const Eigen::VectorXd& opt_params, Eigen::VectorXd& out_full_params) const;
	virtual void BuildFullParams(const Eigen::VectorXd& opt_params, Eigen::VectorXd& out_full_params) const;
	virtual int GetFullParamsSize() const;
};