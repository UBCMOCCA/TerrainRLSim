#pragma once

#include "Shader.h"
#include "util/MathUtil.h"

class cSkyBox
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cSkyBox();
	virtual ~cSkyBox(void);

	virtual void Init();
	virtual void Render() const;

	virtual const tVector& GetSunDirection() const;
	virtual const tVector& GetSunColour() const;
	virtual const float** const GetAmbientColour() const;
	virtual void ChangeSunDirection(const tVector& dir);
	virtual void RotSunDirection(double d_theta);
	virtual void ChangeElevation(double d_theta);
	virtual void SetSunSize(double size);

	virtual void setShaderRelativePath(std::string path);

private:
	static const double PI;
	float mTurbidity;
	float mAlbedo;
	double mHosekSkyCoefs[3][9];
	double mHosekSkyRadiance[3];
	tVector mSunDirection;
	tVector mSunColour;
	double mSunSize;

	// approximate spherical harmonic colours with skylight model
	float mAmbientColour[4][3];

	void UpdateHosekModel();
	void CalculateAmbientLighting();
	void PreethamSky(const tVector& dir, const tVector& sun_dir, 
						float* output );

	float PrezeHosekLuminance( float cos_theta, float cos_gamma, 
						  float A, float B, float C, float D, 
						  float E, float F, float G, float H, float I );
	void XYZ_to_RGB( float X, float Y, float Z, float* output );

	cShader mSkyBoxProgram;

	// shader constants
	GLuint mHosekCoefsHandle;
	GLuint mHosekRadHandle;
	GLuint mSunParamHandle;

	std::string mRelativePath;
};

