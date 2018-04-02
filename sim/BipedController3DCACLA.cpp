/*
 * BipedController3DCACLA.cpp
 *
 *  Created on: Oct 16, 2016
 *      Author: Glen
 */

#include "BipedController3DCACLA.h"

cBipedController3DCACLA::cBipedController3DCACLA() : cTerrainRLCharController(),
cBipedController3D(),
cBaseControllerCacla()
{
	// TODO Auto-generated constructor stub
	mExpParams.mBaseActionRate = 0.2;
	mExpParams.mNoise = 0.2;

}

cBipedController3DCACLA::~cBipedController3DCACLA() {
	// TODO Auto-generated destructor stub
}

void cBipedController3DCACLA::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cBaseControllerCacla::Init(character);
	cBipedController3D::Init(character, gravity, param_file);
}

bool cBipedController3DCACLA::IsCurrActionCyclic() const
{
	return false;
}

void cBipedController3DCACLA::UpdateAction()
{
	cBipedController3D::UpdateAction();
#if defined(ENABLE_DEBUG_VISUALIZATION)
	RecordVal();
#endif // ENABLE_DEBUG_VISUALIZATION
}

