#include "SkyBox.h"
#include <algorithm>
#include "HosekSkyModel.h"
#include "DrawUtil.h"

const double cSkyBox::PI  =3.141592653589793238462;
const double gMinDirY = 0.01;

cSkyBox::cSkyBox() : mTurbidity( 3.f ), mAlbedo( 0.1f )
{
	// the sun is very bright
	mSunColour = tVector(1000, 900, 700, 1);
	mSunDirection = tVector(0.577, 0.277, 0.577, 0).normalized();
	mSunSize = 0.035;
	mRelativePath = "";
}

void cSkyBox::Init()
{
	mSkyBoxProgram.setShaderRelativePath(mRelativePath);
	mSkyBoxProgram.BuildShader("render/shaders/SkyBox_VS.glsl", "render/shaders/SkyBox_PS.glsl");

	// get the constant handles
	mSkyBoxProgram.BindShaderUniform(mHosekCoefsHandle, "gHosekCoefs");
	mSkyBoxProgram.BindShaderUniform(mHosekRadHandle, "gHosekRadiance");
	mSkyBoxProgram.BindShaderUniform(mSunParamHandle, "gSunParam");

	// normalize direction
	float sun_dir_norm = sqrt( mSunDirection[0] * mSunDirection[0] + mSunDirection[1] * mSunDirection[1] + mSunDirection[2] * mSunDirection[2]);
	mSunDirection[0] /= sun_dir_norm;
	mSunDirection[1] /= sun_dir_norm;
	mSunDirection[2] /= sun_dir_norm;

	UpdateHosekModel();
}

void cSkyBox::Render() const
{
	GLint old_cull_face_mode;
    glGetIntegerv(GL_CULL_FACE_MODE, &old_cull_face_mode);
    GLint old_depth_func_mode;
    glGetIntegerv(GL_DEPTH_FUNC, &old_depth_func_mode);

    glCullFace(GL_FRONT);
    glDepthFunc(GL_LEQUAL);
	glDisable(GL_BLEND); // disable alpha blending when drawing sky

	mSkyBoxProgram.Bind();

	// yes.... the hosek model takes a lot of params
	float coefficients[] = {	mHosekSkyCoefs[0][0], mHosekSkyCoefs[0][1], mHosekSkyCoefs[0][2], mHosekSkyCoefs[0][3],
								mHosekSkyCoefs[0][4], mHosekSkyCoefs[0][5], mHosekSkyCoefs[0][6], mHosekSkyCoefs[0][7],
								mHosekSkyCoefs[1][0], mHosekSkyCoefs[1][1], mHosekSkyCoefs[1][2], mHosekSkyCoefs[1][3],
								mHosekSkyCoefs[1][4], mHosekSkyCoefs[1][5], mHosekSkyCoefs[1][6], mHosekSkyCoefs[1][7],
								mHosekSkyCoefs[2][0], mHosekSkyCoefs[2][1], mHosekSkyCoefs[2][2], mHosekSkyCoefs[2][3],
								mHosekSkyCoefs[2][4], mHosekSkyCoefs[2][5], mHosekSkyCoefs[2][6], mHosekSkyCoefs[2][7],
								mHosekSkyCoefs[0][8], mHosekSkyCoefs[1][8], mHosekSkyCoefs[2][8], 0.f };

	float radiance[] = { mHosekSkyRadiance[0], mHosekSkyRadiance[1], mHosekSkyRadiance[2], 0.f };

	float sun_params[] = { mSunDirection[0], mSunDirection[1], mSunDirection[2], mSunSize,
							mSunColour[0], mSunColour[1], mSunColour[2], 0.f };

	glProgramUniform4fv( mSkyBoxProgram.GetProg(), mHosekCoefsHandle, 7, coefficients );
	glProgramUniform4fv( mSkyBoxProgram.GetProg(), mHosekRadHandle, 1, radiance );
	glProgramUniform4fv( mSkyBoxProgram.GetProg(), mSunParamHandle, 2, sun_params );
	
	cDrawUtil::DrawBox(tVector(0, 0, 0, 0), tVector::Ones(), cDrawUtil::eDrawSolid);

	mSkyBoxProgram.Unbind();
	glCullFace(old_cull_face_mode); 
    glDepthFunc(old_depth_func_mode);

	glEnable(GL_BLEND);
}

void cSkyBox::UpdateHosekModel()
{
	printf("Sun Dir: %.5f, %.5f, %.5f\n", mSunDirection[0], mSunDirection[1], mSunDirection[2]);

	float sun_elevation = std::abs( PI / 2 - acosf( mSunDirection[1] )); // model has undefined behavior for elevation < 0
	cHosekSkyModel::CalculateXYZCoefandLM( mTurbidity, mAlbedo, sun_elevation, 
											mHosekSkyCoefs, mHosekSkyRadiance );
	CalculateAmbientLighting();
}

void cSkyBox::CalculateAmbientLighting()
{
	tVector up = tVector(0, 1, 0, 0);
	PreethamSky(up, mSunDirection, mAmbientColour[0]);

	tVector xp = tVector(1, 0, 0, 0);
	PreethamSky(xp, mSunDirection, mAmbientColour[1]);

	tVector zp = tVector(0, 0, 1, 0);
	PreethamSky(zp, mSunDirection, mAmbientColour[2]);

	// down
	// hack hack hack
	float down_scale = 0.4f; // scale the up colour to be the down colour
							// taking into account that the ground will reflect
							// some of the skylight
	mAmbientColour[3][0] = mAmbientColour[0][0] * down_scale;
	mAmbientColour[3][1] = mAmbientColour[0][1] * down_scale;
	mAmbientColour[3][2] = mAmbientColour[0][2] * down_scale;

	//float xn[] = { -1.f, 0.f, 0.f };
	//PreethamSky( xn, mSunDirection, mAmbientColour[3]);

	//float zn[] = { 0.f, 0.f, -1.f };
	//PreethamSky( zn, mSunDirection, mAmbientColour[4]);
}

void cSkyBox::PreethamSky(const tVector& dir, const tVector& sun_dir, float* output )
{
	// all inputs should be float[3]
	float cos_theta = std::abs( sun_dir[1] ); // mode is not defined for cos_theta < 0.f
	float cos_gamma = std::min(1.0, (dir[0] * sun_dir[0]
									+ dir[1] * sun_dir[1]
									+ dir[2] * sun_dir[2]));

	float X = mHosekSkyRadiance[0] * PrezeHosekLuminance( cos_theta, cos_gamma, 
									mHosekSkyCoefs[0][0], mHosekSkyCoefs[0][1], mHosekSkyCoefs[0][2], mHosekSkyCoefs[0][3], 
									mHosekSkyCoefs[0][4], mHosekSkyCoefs[0][5], mHosekSkyCoefs[0][6], mHosekSkyCoefs[0][7],
									mHosekSkyCoefs[0][8] );

	float Y = mHosekSkyRadiance[1] * PrezeHosekLuminance( cos_theta, cos_gamma, 
									mHosekSkyCoefs[1][0], mHosekSkyCoefs[1][1], mHosekSkyCoefs[1][2], mHosekSkyCoefs[1][3], 
									mHosekSkyCoefs[1][4], mHosekSkyCoefs[1][5], mHosekSkyCoefs[1][6], mHosekSkyCoefs[1][7],
									mHosekSkyCoefs[1][8] );

	float Z = mHosekSkyRadiance[2] * PrezeHosekLuminance( cos_theta, cos_gamma, 
									mHosekSkyCoefs[2][0], mHosekSkyCoefs[2][1], mHosekSkyCoefs[2][2], mHosekSkyCoefs[2][3], 
									mHosekSkyCoefs[2][4], mHosekSkyCoefs[2][5], mHosekSkyCoefs[2][6], mHosekSkyCoefs[2][7],
									mHosekSkyCoefs[2][8] );

	XYZ_to_RGB( X, Y, Z, output );

	float scale = ( 0.1f + 0.9f * cos_theta );
	output[0] *= scale;
	output[1] *= scale;
	output[2] *= scale;
}

void cSkyBox::XYZ_to_RGB( float X, float Y, float Z, float* output )
{
	output[0] = 3.240479f * X + -1.537150f * Y + -0.498535f * Z;
	output[1] = -0.969256f * X + 1.875992f * Y + 0.041556f * Z;
	output[2] = 0.055648f * X + -0.204043f * Y + 1.057311f * Z;
}


float cSkyBox::PrezeHosekLuminance( float cos_theta, float cos_gamma, 
						  float A, float B, float C, float D, 
						  float E, float F, float G, float H, float I )
{
	float gamma = std::acos( cos_gamma );
	float expM = std::exp(E * gamma);
    float rayM = std::cos(gamma)*std::cos(gamma);
    float mieM = (1.0 + std::cos(gamma)*std::cos(gamma)) 
					/ std::pow( std::abs( 1.0 + I * I - 2.0* I * std::cos( gamma )), 1.5);
    float zenith = std::sqrt( cos_theta );

    return (1.0 + A * std::exp( B / ( cos_theta + 0.01 ))) *
            ( C + D * expM + F * rayM + G * mieM + H * zenith);
}

const tVector& cSkyBox::GetSunDirection() const
{
	return mSunDirection;
}

const tVector& cSkyBox::GetSunColour() const
{
	return mSunColour;
}

const float** const cSkyBox::GetAmbientColour() const
{
	return (const float** const) mAmbientColour;
}

void cSkyBox::ChangeSunDirection( const tVector& dir )
{
	mSunDirection = dir;
	mSunDirection[1] = std::max(gMinDirY, mSunDirection[1]);
	mSunDirection.normalize();

	UpdateHosekModel();
}

void cSkyBox::RotSunDirection(double d_theta)
{
	tMatrix mat = cMathUtil::RotateMat(tVector(0, 1, 0, 0), d_theta);
	mSunDirection = mat * mSunDirection;

	UpdateHosekModel();
}

void cSkyBox::ChangeElevation(double d_theta)
{
	tVector tangent = tVector(0, 1, 0, 0).cross3(mSunDirection);
	tangent = tangent.normalized();

	tMatrix mat = cMathUtil::RotateMat(tangent, d_theta);
	mSunDirection = mat * mSunDirection;
	mSunDirection[1] = std::max(gMinDirY, mSunDirection[1]);
	mSunDirection.normalize();

	UpdateHosekModel();
}

void cSkyBox::SetSunSize(double size)
{
	mSunSize = size;
}

cSkyBox::~cSkyBox(void)
{
}

void cSkyBox::setShaderRelativePath(std::string path)
{
	this->mRelativePath = path;
}
