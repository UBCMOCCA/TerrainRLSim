/*
 * BipedController3DCACLA.h
 *
 *  Created on: Oct 16, 2016
 *      Author: Glen
 */

#ifndef SIM_BIPEDCONTROLLER3DCACLA_H_
#define SIM_BIPEDCONTROLLER3DCACLA_H_

#include "sim/BaseControllerCacla.h"
#include "sim/BipedController3D.h"

class cBipedController3DCACLA : public cBipedController3D, public virtual cBaseControllerCacla{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cBipedController3DCACLA();
	virtual ~cBipedController3DCACLA();

	virtual void Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file);

protected:
	virtual void UpdateAction();
	virtual bool IsCurrActionCyclic() const;
};

#endif /* SIM_BIPEDCONTROLLER3DCACLA_H_ */
