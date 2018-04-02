/*
 * BaseControllerCaclaFD.h
 *
 *  Created on: Nov 18, 2016
 *      Author: jocst
 */

#ifndef SIM_BASECONTROLLERCACLAFD_H_
#define SIM_BASECONTROLLERCACLAFD_H_

#include "BaseControllerCacla.h"

class cBaseControllerCaclaFD : public cBaseControllerCacla {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cBaseControllerCaclaFD();
	virtual ~cBaseControllerCaclaFD();

	virtual bool LoadForwardDynamicsNet(const std::string& net_file);
	virtual void LoadForwardDynamicsModel(const std::string& model_file);

	virtual void Init(cSimCharacter* character);
	virtual void BuildForwardDynamicsNet(std::unique_ptr<cNeuralNet>& out_net) const;

	virtual void BuildForwardDynamicsInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildForwardDynamicsOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void SetForwardDynamicsInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const;
	virtual void SetForwardDynamicsOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const;

	virtual int GetForwardDynamicsInputSize() const;
	virtual int GetForwardDynamicsOutputSize() const;
	virtual void CopyForwardDynamicsNet(const cNeuralNet& net);
	virtual const std::unique_ptr<cNeuralNet>& GetForwardDynamics() const;

	virtual void ExploitPolicy(const Eigen::VectorXd& state, tAction& out_action, Eigen::VectorXd& out_params);

private:
	std::unique_ptr<cNeuralNet> mForwardDynamicsNet;

	virtual void ApplyExpNoise(tAction& out_action, Eigen::VectorXd & in_action_params);
	virtual void ComputeActionGradient(Eigen::VectorXd & in_action, Eigen::VectorXd& out_action_grad);
	virtual void ExploreAction(Eigen::VectorXd& state, tAction& out_action);

};

#endif /* SIM_BASECONTROLLERCACLAFD_H_ */
