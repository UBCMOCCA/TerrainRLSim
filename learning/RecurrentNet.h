#pragma once
#include "NeuralNet.h"

class cRecurrentNet : public cNeuralNet
{
public:
	cRecurrentNet();
	virtual ~cRecurrentNet();

	virtual int GetInputSize() const;
	virtual void Eval(const Eigen::VectorXd& x, bool is_start, Eigen::VectorXd& out_y) const;
	
	virtual int GetNumStreams() const;
	virtual int GetProblemXSize() const;
	virtual int GetProblemYSize() const;

protected:

	virtual void LoadTrainData(const Eigen::MatrixXd& X, const Eigen::MatrixXd& Y);
};