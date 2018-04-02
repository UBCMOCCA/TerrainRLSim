#pragma once
#include "sim/CtTrackController.h"
#include "sim/CtMTUController.h"

class cCtMTUTrackController : public virtual cCtTrackController, public virtual cCtMTUController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cCtMTUTrackController();
	virtual ~cCtMTUTrackController();

	virtual void Init(cSimCharacter* character, const std::string& param_file);
	virtual int GetPoliStateSize() const;

protected:

	virtual int GetTargetStateOffset() const;
	virtual void BuildPoliState(Eigen::VectorXd& out_state) const;
};