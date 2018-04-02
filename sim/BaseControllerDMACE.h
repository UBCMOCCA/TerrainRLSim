#pragma once

#include "sim/BaseControllerMACE.h"

class cBaseControllerDMACE : public virtual cBaseControllerMACE
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cBaseControllerDMACE();
	virtual ~cBaseControllerDMACE();

	virtual void Init(cSimCharacter* character);

	virtual bool ValidCritic() const;
	virtual bool LoadCriticNet(const std::string& net_file);
	virtual void LoadCriticModel(const std::string& model_file);

	virtual void CopyNet(const cNeuralNet& net);
	virtual void CopyActorNet(const cNeuralNet& net);
	virtual void CopyCriticNet(const cNeuralNet& net);

	virtual const std::unique_ptr<cNeuralNet>& GetActor() const;
	virtual const std::unique_ptr<cNeuralNet>& GetCritic() const;

	virtual int GetActorInputSize() const;
	virtual int GetActorOutputSize() const;
	virtual int GetCriticInputSize() const;
	virtual int GetCriticOutputSize() const;

	virtual void BuildNNInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void SetNNInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const;
	virtual void SetNNOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const;

	virtual void BuildActorInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void SetActorInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const;
	virtual void SetActorOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const;

	virtual void BuildCriticInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildCriticOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void SetCriticInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const;
	virtual void SetCriticOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const;
	
protected:
	std::unique_ptr<cNeuralNet> mCriticNet;

	virtual void BuildCriticNet(std::unique_ptr<cNeuralNet>& out_net) const;
	virtual void RecordVal(double val);
	virtual void BuildCriticInput(Eigen::VectorXd& out_x) const;

#if defined(ENABLE_DEBUG_VISUALIZATION)
public:
	virtual void GetVisActionValues(Eigen::VectorXd& out_vals) const;
#endif
};