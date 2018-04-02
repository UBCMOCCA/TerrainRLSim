#pragma once
#include "sim/CtTrackController.h"
#include "sim/CtNPDController.h"

class cCtNPDTrackController : public virtual cCtTrackController, public virtual cCtNPDController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cCtNPDTrackController();
	virtual ~cCtNPDTrackController();

	virtual void Init(cSimCharacter* character);
	virtual int GetPoliStateSize() const;

protected:

	virtual int GetPDStateOffset() const;
	virtual void BuildPoliState(Eigen::VectorXd& out_state) const;

};