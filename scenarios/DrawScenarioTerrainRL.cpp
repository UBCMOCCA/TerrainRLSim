#include "DrawScenarioTerrainRL.h"
#include "render/DrawUtil.h"
#include "render/DrawObj.h"
#include "render/DrawWorld.h"
#include "render/DrawCharacter.h"
#include "render/DrawPerturb.h"
#include "render/DrawGround.h"
#include "render/Shader.h"
#include "render/SkyBox.h"
#include "render/PostProcessor.h"
#include "render/TextureDesc.h"
#include "render/ShadowMap.h"
#include "render/GBuffer.h"
#include <string>
#include <iostream>

const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gVisOffset = tVector(0, 0, 0.5, 0); // offset for visualization elements

cDrawScenarioTerrainRL::cDrawScenarioTerrainRL(cCamera& cam)
						: cDrawScenario(cam)
{
	mDraw3D = false;
	mClearFilmStrip = false;
	mCamDelta = cam.GetPosition() - cam.GetFocus();
}

cDrawScenarioTerrainRL::~cDrawScenarioTerrainRL()
{
}

void cDrawScenarioTerrainRL::Init()
{
	cDrawScenario::Init();

	InitRenderResources();
	mClearFilmStrip = false;
	EnableDraw3D(mDraw3D);
}

void cDrawScenarioTerrainRL::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cDrawScenario::ParseArgs(parser);
	mArgParser = parser;
}

void cDrawScenarioTerrainRL::Reset()
{
	cDrawScenario::Reset();
	mClearFilmStrip = false;
}

void cDrawScenarioTerrainRL::Clear()
{
	cDrawScenario::Clear();
}

void cDrawScenarioTerrainRL::Update(double time_elapsed)
{
	cDrawScenario::Update(time_elapsed);
	UpdateScene(time_elapsed);

	tVector prev_cam_focus = mCam.GetFocus();
	UpdateCamera();

	if (mEnableFilmStrip)
	{
		if (!(prev_cam_focus - mCam.GetFocus()).isMuchSmallerThan(0.001))
		{
			mClearFilmStrip = true;
		}
	}
}

tVector cDrawScenarioTerrainRL::GetCamTrackPos() const
{
	return mCam.GetFocus();
}

tVector cDrawScenarioTerrainRL::GetCamStillPos() const
{
	return mCam.GetFocus();
}

void cDrawScenarioTerrainRL::ResetCallback()
{
	ResetCamera();
}

void cDrawScenarioTerrainRL::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenario::Keyboard(key, x, y);

	switch (key)
	{
	case 'c':
		ToggleCamTrackMode(eCamTrackModeStill);
		break;
	case '\r':
		ToggleDraw3D();
		break;
	case 'w':
		mSkyBox->RotSunDirection(-0.01);
		break;
	case 'e':
		mSkyBox->RotSunDirection(0.01);
		break;
	case 'W':
		mSkyBox->ChangeElevation(-0.01);
		break;
	case 'E':
		mSkyBox->ChangeElevation(0.01);
		break;
	default:
		break;
	}
}

void cDrawScenarioTerrainRL::MouseClick(int button, int state, double x, double y)
{
	cDrawScenario::MouseClick(button, state, x, y);
	eDrawMode draw_mode = GetDrawMode();
	if (draw_mode == eDrawModeUber3D 
		|| draw_mode == eDrawMode3D)
	{
		mCam.MouseClick(button, state, x, y);
	}
}

void cDrawScenarioTerrainRL::MouseMove(double x, double y)
{
	cDrawScenario::MouseMove(x, y);
	eDrawMode draw_mode = GetDrawMode();
	if (draw_mode == eDrawModeUber3D
		|| draw_mode == eDrawMode3D)
	{
		mCam.MouseMove(x, y);
	}
}

void cDrawScenarioTerrainRL::Reshape(int w, int h)
{
	mIntBufferTex->Reshape(w, h);
	mPostProcessor->Reshape(w, h);
	//mDepthTex->Reshape(w, h);
	//mAOTex->Reshape(w, h);
	//mGBuffer->Reshape(w, h);
	//mReflectionTex->Reshape(w, h);
	//mReflectionTex1->Reshape(w, h);
}

void cDrawScenarioTerrainRL::SetOutputTex(const std::shared_ptr<cTextureDesc>& tex)
{
	mOutputTex = tex;
}

cDrawScenarioTerrainRL::eDrawMode cDrawScenarioTerrainRL::GetDrawMode() const
{
	eDrawMode draw_mode = eDrawMode2D;
	if (mDraw3D)
	{
		draw_mode = eDrawMode3D;
	}
	return draw_mode;
}

void cDrawScenarioTerrainRL::DrawScene()
{
	mOutputTex->BindBuffer();
	mCam.SetupGLProj();

	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);

	eDrawMode draw_mode = GetDrawMode();
	if (draw_mode == eDrawModeUber3D)
	{
		DoShadowPass();
		//DoDepthPass();
		//DoAmbientOcclusion();
		//DoGeoPass();
		mIntBufferTex->BindBuffer();

		bool clear_frame = !mEnableFilmStrip || mClearFilmStrip;
		if (clear_frame)
		{
			cDrawUtil::ClearColor(tVector(0, 0, 0, 0));
			cDrawUtil::ClearDepth(1);

			if (mEnableFilmStrip)
			{
				mClearFilmStrip = false;
			}
		}

		DrawSky();

		mShaderMesh->Bind();
		SetupMeshShaderUber3D();

		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
	}
	else if (draw_mode == eDrawMode2D)
	{
		DrawGrid();
	}
	else if (draw_mode == eDrawMode3D)
	{
		DoShadowPass();
		mOutputTex->BindBuffer();
		mLambertShaderMesh->Bind();
		SetupMeshShader3D();

		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
	}

	DrawGroundMainScene();
	DrawCharacterMainScene();
	DrawObjsMainScene();
	DrawMiscMainScene();

	if (draw_mode == eDrawModeUber3D)
	{
		mShaderMesh->Unbind();
		mIntBufferTex->UnbindBuffer();
		//DoReflectionPass();
		DoPostProcess(*mIntBufferTex, *mOutputTex);
	}
	else if (draw_mode == eDrawMode3D)
	{
		mLambertShaderMesh->Unbind();
	}

	DrawInfo();

	mOutputTex->UnbindBuffer();
}

void cDrawScenarioTerrainRL::DrawGrid() const
{
	const double spacing = 0.10f;
	const double big_spacing = spacing * 5.f;
	tVector origin = mCam.GetFocus();
	origin += tVector(0, 0, -1, 0);
	tVector size = tVector(mCam.GetWidth(), mCam.GetHeight(), 0, 0);

	cDrawUtil::SetColor(tVector(188 / 255.f, 219 / 255.f, 242 / 255.f, 1.f));
	cDrawUtil::DrawGrid2D(origin, size, spacing, big_spacing);
}

void cDrawScenarioTerrainRL::DrawGroundMainScene()
{
	eDrawMode draw_mode = GetDrawMode();
	const double roughness = 0.5;
	const double enable_tex = 1;
	if (draw_mode == eDrawModeUber3D)
	{
		mShaderMesh->SetUniform4(mMaterialDataHandle, tVector(roughness, enable_tex, 0, 0));
		mGridTex0->BindTex(GL_TEXTURE0);
	}
	else if (draw_mode == eDrawMode3D)
	{
		mLambertShaderMesh->SetUniform4(mLambertMaterialDataHandle, tVector(roughness, enable_tex, 0, 0));
		mGridTex1->BindTex(GL_TEXTURE0);
	}

	DrawGround();

	if (draw_mode == eDrawModeUber3D)
	{
		mGridTex0->UnbindTex(GL_TEXTURE0);
	}
	else if (draw_mode == eDrawMode3D)
	{
		mGridTex1->UnbindTex(GL_TEXTURE0);
	}
}

void cDrawScenarioTerrainRL::DrawCharacterMainScene()
{
	eDrawMode draw_mode = GetDrawMode();
	const double roughness = 0.4;
	const double enable_tex = 0;
	if (draw_mode == eDrawModeUber3D)
	{
		mShaderMesh->SetUniform4(mMaterialDataHandle, tVector(roughness, enable_tex, 0, 0));
	}
	else if (draw_mode == eDrawMode3D)
	{
		mLambertShaderMesh->SetUniform4(mLambertMaterialDataHandle, tVector(roughness, enable_tex, 0, 0));
	}
	DrawCharacters();
}

void cDrawScenarioTerrainRL::DrawObjsMainScene()
{
	eDrawMode draw_mode = GetDrawMode();
	const double roughness = 0.4;
	const double enable_tex = 0;
	if (draw_mode == eDrawModeUber3D)
	{
		mShaderMesh->SetUniform4(mMaterialDataHandle, tVector(roughness, enable_tex, 0, 0));
	}
	else if (draw_mode == eDrawMode3D)
	{
		mLambertShaderMesh->SetUniform4(mLambertMaterialDataHandle, tVector(roughness, enable_tex, 0, 0));
	}
	DrawObjs();
}

void cDrawScenarioTerrainRL::DrawMiscMainScene()
{
	eDrawMode draw_mode = GetDrawMode();
	const double roughness = 0.4;
	const double enable_tex = 0;
	if (draw_mode == eDrawModeUber3D)
	{
		mShaderMesh->SetUniform4(mMaterialDataHandle, tVector(roughness, enable_tex, 0, 0));
	}
	else if (draw_mode == eDrawMode3D)
	{
		mLambertShaderMesh->SetUniform4(mLambertMaterialDataHandle, tVector(roughness, enable_tex, 0, 0));
	}
	DrawMisc();
}

void cDrawScenarioTerrainRL::DrawGround() const
{
}

void cDrawScenarioTerrainRL::DrawCharacters() const
{
}

void cDrawScenarioTerrainRL::DrawObjs() const
{
}

void cDrawScenarioTerrainRL::DrawMisc() const
{
}

void cDrawScenarioTerrainRL::DrawInfo() const
{
}

void cDrawScenarioTerrainRL::ToggleDraw3D()
{
	EnableDraw3D(!mDraw3D);
}

void cDrawScenarioTerrainRL::EnableDraw3D(bool enable)
{
	mDraw3D = enable;
	eDrawMode draw_mode = GetDrawMode();
	if (draw_mode == eDrawModeUber3D || draw_mode == eDrawMode3D)
	{
		mCam.SetProj(cCamera::eProjPerspective);
	}
	else
	{
		mCam.SetProj(cCamera::eProjOrtho);
		mCam.SetPosition(mCam.GetFocus() + mCamDelta);
		mCam.SetUp(tVector(0, 1, 0, 0));
	}
	//ToggleCamTrackMode(eCamTrackModeXYZ);
}

tVector cDrawScenarioTerrainRL::GetVisOffset() const
{
	eDrawMode draw_mode = GetDrawMode();
	if (draw_mode == eDrawModeUber3D
		|| draw_mode == eDrawMode3D)
	{
		return tVector::Zero();
	}
	return gVisOffset;
}

tVector cDrawScenarioTerrainRL::GetLineColor() const
{
	eDrawMode draw_mode = GetDrawMode();
	if (draw_mode == eDrawModeUber3D)
	{
		return tVector::Zero();
	}
	return gLineColor;
}

tVector cDrawScenarioTerrainRL::GetGroundColor() const
{
	eDrawMode draw_mode = GetDrawMode();
	if (draw_mode == eDrawModeUber3D
		|| draw_mode == eDrawMode3D)
	{
		return tVector::Ones();
	}
	else
	{
		return tVector(151 / 255.0, 151 / 255.0, 151 / 255.0, 1.0);
	}
}

std::string cDrawScenarioTerrainRL::BuildTextInfoStr() const
{
	return "";
}

void cDrawScenarioTerrainRL::Shutdown()
{
}

void cDrawScenarioTerrainRL::InitRenderResources()
{
	bool succ = true;

	{
		mShaderMesh = std::unique_ptr<cShader>(new cShader());
		std::string fpath;
		bool succ_ = mArgParser->ParseString("relative_file_path", fpath);
		if (succ_)
		{
			mShaderMesh->setShaderRelativePath(fpath);
		}
		succ &= mShaderMesh->BuildShader("render/shaders/Mesh_VS.glsl", "render/shaders/Lighting_PS.glsl");

		mShaderMesh->Bind();
		mShaderMesh->BindShaderUniform(mLightDirHandle, "gLightDir");
		mShaderMesh->BindShaderUniform(mLightColourHandle, "gLightColour");
		mShaderMesh->BindShaderUniform(mAmbientColourHandle, "gAmbientColour");
		mShaderMesh->BindShaderUniform(mShadowProjHandle, "gShadowProj");
		mShaderMesh->BindShaderUniform(mMaterialDataHandle, "gMaterialData");
		mShaderMesh->BindShaderUniform(mTexResHandle, "gTexRes");

		GLint albedo_tex = glGetUniformLocation(mShaderMesh->GetProg(), "gTexture");
		glUniform1i(albedo_tex, 0);
		GLint shadow_tex = glGetUniformLocation(mShaderMesh->GetProg(), "gShadowTex");
		glUniform1i(shadow_tex, 1);
		GLint ao_tex = glGetUniformLocation(mShaderMesh->GetProg(), "gAOTex");
		glUniform1i(ao_tex, 2);
		mShaderMesh->Unbind();
	}

	{
		mShaderDepth = std::unique_ptr<cShader>(new cShader());
		std::string fpath;
		bool succ_ = mArgParser->ParseString("relative_file_path", fpath);
		if (succ_)
		{
			mShaderDepth->setShaderRelativePath(fpath);
		}
		succ &= mShaderDepth->BuildShader("render/shaders/Mesh_VS.glsl", "render/shaders/Depth_PS.glsl");
	}

	{
		mShaderGBuffer = std::unique_ptr<cShader>(new cShader());
		std::string fpath;
		bool succ_ = mArgParser->ParseString("relative_file_path", fpath);
		if (succ_)
		{
			mShaderGBuffer->setShaderRelativePath(fpath);
		}
		succ &= mShaderGBuffer->BuildShader("render/shaders/Mesh_VS.glsl", "render/shaders/GBuffer_PS.glsl");
	}

	{
		mLambertShaderMesh = std::unique_ptr<cShader>(new cShader());
		std::string fpath;
		bool succ_ = mArgParser->ParseString("relative_file_path", fpath);
		if (succ_)
		{
			mLambertShaderMesh->setShaderRelativePath(fpath);
		}
		succ &= mLambertShaderMesh->BuildShader("render/shaders/Mesh_VS.glsl", "render/shaders/Lighting_Lambert_PS.glsl");

		mLambertShaderMesh->Bind();
		mLambertShaderMesh->BindShaderUniform(mLambertLightDirHandle, "gLightDir");
		mLambertShaderMesh->BindShaderUniform(mLambertLightColourHandle, "gLightColour");
		mLambertShaderMesh->BindShaderUniform(mLambertAmbientColourHandle, "gAmbientColour");
		mLambertShaderMesh->BindShaderUniform(mLambertShadowProjHandle, "gShadowProj");
		mLambertShaderMesh->BindShaderUniform(mLambertMaterialDataHandle, "gMaterialData");
		mLambertShaderMesh->BindShaderUniform(mLambertFogColorHandle, "gFogColor");
		mLambertShaderMesh->BindShaderUniform(mLambertFogDataHandle, "gFogData");

		GLint albedo_tex = glGetUniformLocation(mLambertShaderMesh->GetProg(), "gTexture");
		glUniform1i(albedo_tex, 0);
		GLint shadow_tex = glGetUniformLocation(mLambertShaderMesh->GetProg(), "gShadowTex");
		glUniform1i(shadow_tex, 1);
		mLambertShaderMesh->Unbind();
	}

	/*
	{
		mShaderHBAO = std::unique_ptr<cShader>(new cShader());
		std::string fpath;
		bool succ_ = mArgParser->ParseString("relative_file_path", fpath);
		if (succ_)
		{
			mShaderHBAO->setShaderRelativePath(fpath);
		}
		succ &= mShaderHBAO->BuildShader("render/shaders/FullScreenQuad_VS.glsl", "render/shaders/HBAO_PS.glsl");

		mShaderHBAO->Bind();
		GLint depth_tex = glGetUniformLocation(mShaderHBAO->GetProg(), "gDepthTex");
		glUniform1i(depth_tex, 0);

		mShaderHBAO->BindShaderUniform(mHBAOFocalLenLinMADHandle, "FocalLenLinMAD");
		mShaderHBAO->BindShaderUniform(mHBAOUVToViewABHandle, "UVToViewAB");
		mShaderHBAO->BindShaderUniform(mHBAOTexResHandle, "TexRes");
		mShaderHBAO->Unbind();
	}
	*/

	/*
	{
		mShaderSSR = std::unique_ptr<cShader>(new cShader());
		std::string fpath;
		bool succ_ = mArgParser->ParseString("relative_file_path", fpath);
		if (succ_)
		{
			mShaderSSR->setShaderRelativePath(fpath);
		}
		succ &= mShaderSSR->BuildShader("render/shaders/FullScreenQuad_VS.glsl", "render/shaders/SSR_PS.glsl");

		mShaderSSR->Bind();
		GLint depth_tex = glGetUniformLocation(mShaderSSR->GetProg(), "gDepthTex");
		glUniform1i(depth_tex, 0);
		GLint norm_tex = glGetUniformLocation(mShaderSSR->GetProg(), "gNormalTex");
		glUniform1i(norm_tex, 1);
		GLint screen_tex = glGetUniformLocation(mShaderSSR->GetProg(), "gScreenTex");
		glUniform1i(screen_tex, 2);
		GLint depth_back_tex = glGetUniformLocation(mShaderSSR->GetProg(), "gDepthBackTex");
		glUniform1i(depth_back_tex, 3);

		mShaderSSR->BindShaderUniform(mSSRFocalLenLinMADHandle, "FocalLenLinMAD");
		mShaderSSR->BindShaderUniform(mSSRUVToViewABHandle, "UVToViewAB");
		mShaderSSR->BindShaderUniform(mSSRTexResHandle, "TexRes");
		mShaderSSR->Unbind();
	}

	{
		mShaderApplySSR = std::unique_ptr<cShader>(new cShader());
		std::string fpath;
		bool succ_ = mArgParser->ParseString("relative_file_path", fpath);
		if (succ_)
		{
			mShaderApplySSR->setShaderRelativePath(fpath);
		}
		succ &= mShaderApplySSR->BuildShader("render/shaders/FullScreenQuad_VS.glsl", "render/shaders/ApplySSR_PS.glsl");

		mShaderApplySSR->Bind();
		GLint refl_tex = glGetUniformLocation(mShaderApplySSR->GetProg(), "gReflectionTex");
		glUniform1i(refl_tex, 0);
		mShaderApplySSR->Unbind();
	}
	*/

	mSkyBox = std::unique_ptr<cSkyBox>(new cSkyBox());
	std::string fpath;
	bool succ_ = mArgParser->ParseString("relative_file_path", fpath);
	if (succ_)
	{
		mSkyBox->setShaderRelativePath(fpath);
	}
	mSkyBox->Init();
	mSkyBox->ChangeSunDirection(tVector(0.6, 0.67, 0.45, 0));
	mSkyBox->SetSunSize(0.01);

	int w = mOutputTex->GetWidth();
	int h = mOutputTex->GetHeight();
	mPostProcessor = std::unique_ptr<cPostProcessor>(new cPostProcessor());
	// std::string fpath;
	succ_ = mArgParser->ParseString("relative_file_path", fpath);
	if (succ_)
	{
		mPostProcessor->setShaderRelativePath(fpath);
		// mShaderApplySSR->setShaderRelativePath(fpath);
	}
	mPostProcessor->Init(w, h);
	mIntBufferTex = std::unique_ptr<cTextureDesc>(new cTextureDesc(w, h, GL_RGBA16F, GL_RGBA, GL_FLOAT, false));
	
	//float shadow_size = 20.f;
	//float shadow_near_z = 1.f;
	//float shadow_far_z = 40.f;
	//int shadow_res = 1024;
	float shadow_size = 40.f;
	float shadow_near_z = 1.f;
	float shadow_far_z = 60.f;
	int shadow_res = 2048;
	mShadowCam = cCamera(tVector(0, 0, 1, 0), tVector::Zero(), tVector(0, 1, 0, 0),
						shadow_size, shadow_size, shadow_near_z, shadow_far_z);
	mShadowCam.SetProj(cCamera::eProjOrtho);
	mShadowMap = std::unique_ptr<cShadowMap>(new cShadowMap());
	mShadowMap->Init(shadow_res, shadow_res);

	//mDepthTex = std::unique_ptr<cShadowMap>(new cShadowMap());
	//mDepthTex->Init(w, h);

	//mGBuffer = std::unique_ptr<cGBuffer>(new cGBuffer());
	//mGBuffer->Init(w, h);

	//mReflectionTex = std::unique_ptr<cTextureDesc>(new cTextureDesc(w, h, GL_RGB16F, GL_RGB, GL_FLOAT, false));
	//mReflectionTex1 = std::unique_ptr<cTextureDesc>(new cTextureDesc(w, h, GL_RGB16F, GL_RGB, GL_FLOAT, false));

	//mAOTex = std::unique_ptr<cTextureDesc>(new cTextureDesc(w, h, GL_R8, GL_RED, GL_UNSIGNED_BYTE, false));
	
	succ &= LoadTextures();
	
	if (!succ)
	{
		printf("Failed to setup render resources\n");
	}
}

bool cDrawScenarioTerrainRL::LoadTextures()
{
	bool succ = true;
	std::string fpath;
	bool succ_ = mArgParser->ParseString("relative_file_path", fpath);
	if (succ_)
	{
		mPostProcessor->setShaderRelativePath(fpath);
		// mShaderApplySSR->setShaderRelativePath(fpath);
		std::cout << "texture path: " << fpath + "data/textures/grid0.png" << std::endl;
		mGridTex0 = std::unique_ptr<cTextureDesc>(new cTextureDesc(fpath + "data/textures/grid0.png", true));
		succ &= mGridTex0->IsValid();

		mGridTex1 = std::unique_ptr<cTextureDesc>(new cTextureDesc(fpath + "data/textures/grid1.png", true));
		succ &= mGridTex1->IsValid();
	}
	else
	{
		mGridTex0 = std::unique_ptr<cTextureDesc>(new cTextureDesc("data/textures/grid0.png", true));
		succ &= mGridTex0->IsValid();

		mGridTex1 = std::unique_ptr<cTextureDesc>(new cTextureDesc("data/textures/grid1.png", true));
		succ &= mGridTex1->IsValid();
	}

	return succ;
}

void cDrawScenarioTerrainRL::SetupMeshShaderUber3D()
{
	tMatrix view_mat = mCam.BuildWorldViewMatrix();

	tVector light_dir = mSkyBox->GetSunDirection();
	light_dir = view_mat * light_dir;

	const float sun_scale = 0.002f;
	tVector light_col = mSkyBox->GetSunColour();
	light_col *= sun_scale;

	const float** const ambient_colour = mSkyBox->GetAmbientColour();

	double w = static_cast<double>(mIntBufferTex->GetWidth());
	double h = static_cast<double>(mIntBufferTex->GetHeight());
	tVector tex_res = tVector(w, h, 0, 0);

	mShaderMesh->SetUniform3(mLightDirHandle, light_dir);
	mShaderMesh->SetUniform3(mLightColourHandle, light_col);
	mShaderMesh->SetUniform4(mTexResHandle, tex_res);
	glProgramUniform3fv(mShaderMesh->GetProg(), mAmbientColourHandle, 4, (float*)ambient_colour);
	
	tMatrix view_world = mCam.BuildViewWorldMatrix();
	tMatrix shadow_view = mShadowCam.BuildWorldViewMatrix();
	tMatrix shadow_proj = mShadowCam.BuildProjMatrix();
	tMatrix shadow_mat = shadow_proj * shadow_view * view_world;

	float shadow_mat_data[] = { (float)shadow_mat(0, 0), (float)shadow_mat(1, 0), (float)shadow_mat(2, 0), (float)shadow_mat(3, 0),
							(float)shadow_mat(0, 1), (float)shadow_mat(1, 1), (float)shadow_mat(2, 1), (float)shadow_mat(3, 1),
							(float)shadow_mat(0, 2), (float)shadow_mat(1, 2), (float)shadow_mat(2, 2), (float)shadow_mat(3, 2),
							(float)shadow_mat(0, 3), (float)shadow_mat(1, 3), (float)shadow_mat(2, 3), (float)shadow_mat(3, 3)};
	glProgramUniformMatrix4fv(mShaderMesh->GetProg(), mShadowProjHandle, 1, false, shadow_mat_data);

	mShadowMap->BindTex(GL_TEXTURE1);
	//mAOTex->BindTex(GL_TEXTURE2);
}

void cDrawScenarioTerrainRL::SetupMeshShader3D()
{
	tMatrix view_mat = mCam.BuildWorldViewMatrix();

	tVector cam_focus = mCam.GetFocus();
	tVector cam_pos = mCam.GetPosition();
	double near_dist = mCam.GetNearZ();

	const double fog_cutoff = (cam_focus - cam_pos).norm() - near_dist;
	const double fog_decay = 0.001;
	const tVector ambient_col = tVector(0.6, 0.6, 0.6, 0);
	const tVector light_col = tVector(0.5, 0.5, 0.5, 0);
	const tVector fog_col = tVector(0.97, 0.97, 1, 1);
	const tVector fog_data = tVector(fog_cutoff, fog_decay, 0, 0);

	tVector light_dir = mSkyBox->GetSunDirection();
	light_dir = view_mat * light_dir;

	double w = static_cast<double>(mIntBufferTex->GetWidth());
	double h = static_cast<double>(mIntBufferTex->GetHeight());
	tVector tex_res = tVector(w, h, 0, 0);

	mLambertShaderMesh->SetUniform3(mLambertLightDirHandle, light_dir);
	mLambertShaderMesh->SetUniform3(mLambertLightColourHandle, light_col);
	mLambertShaderMesh->SetUniform3(mLambertAmbientColourHandle, ambient_col);
	mLambertShaderMesh->SetUniform4(mLambertFogColorHandle, fog_col);
	mLambertShaderMesh->SetUniform4(mLambertFogDataHandle, fog_data);

	tMatrix view_world = mCam.BuildViewWorldMatrix();
	tMatrix shadow_view = mShadowCam.BuildWorldViewMatrix();
	tMatrix shadow_proj = mShadowCam.BuildProjMatrix();
	tMatrix shadow_mat = shadow_proj * shadow_view * view_world;

	float shadow_mat_data[] = { (float)shadow_mat(0, 0), (float)shadow_mat(1, 0), (float)shadow_mat(2, 0), (float)shadow_mat(3, 0),
		(float)shadow_mat(0, 1), (float)shadow_mat(1, 1), (float)shadow_mat(2, 1), (float)shadow_mat(3, 1),
		(float)shadow_mat(0, 2), (float)shadow_mat(1, 2), (float)shadow_mat(2, 2), (float)shadow_mat(3, 2),
		(float)shadow_mat(0, 3), (float)shadow_mat(1, 3), (float)shadow_mat(2, 3), (float)shadow_mat(3, 3) };
	glProgramUniformMatrix4fv(mLambertShaderMesh->GetProg(), mLambertShadowProjHandle, 1, false, shadow_mat_data);

	mShadowMap->BindTex(GL_TEXTURE1);
	//mAOTex->BindTex(GL_TEXTURE2);
}

void cDrawScenarioTerrainRL::DrawSky()
{
	mSkyBox->Render();
}

void cDrawScenarioTerrainRL::DoPostProcess(const cTextureDesc& src_tex, const cTextureDesc& dst_tex)
{
	tVector sun_dir = mSkyBox->GetSunDirection();

	tMatrix model_view = mCam.BuildWorldViewMatrix();
	tMatrix proj = mCam.BuildProjMatrix();

	tVector sun_coord = (proj * model_view) * sun_dir;
	sun_coord[0] = sun_coord[0] * 0.5 + 0.5;
	sun_coord[1] = sun_coord[1] * 0.5 + 0.5;

	mPostProcessor->SetSunScreenPos(sun_coord);
	mPostProcessor->DoPostProcessing(src_tex, dst_tex);
	//cDrawUtil::CopyTexture(*mShadowMap, dst_tex);
	//cDrawUtil::CopyTexture(src_tex, dst_tex);
	//cDrawUtil::CopyTexture(*mReflectionTex, dst_tex);
}

void cDrawScenarioTerrainRL::DoShadowPass()
{
	// front face culling to prevent shelf occlusion
	glCullFace(GL_FRONT);

	//float dist = 10.f;
	float dist = 30.f;
	const tVector& sun_dir = mSkyBox->GetSunDirection();
	tVector delta = dist * sun_dir;
	tVector focus = mCam.GetFocus();
	mShadowCam.SetPosition(focus + delta);
	mShadowCam.SetFocus(focus);

	mShadowMap->BindBuffer();
	mShaderDepth->Bind();
	cDrawUtil::ClearColor(tVector(1, 1, 1, 0));
	cDrawUtil::ClearDepth(1);

	// shadow pass
	glMatrixMode(GL_PROJECTION);
	cDrawUtil::PushMatrix();
	mShadowCam.SetupGLProj();

	glMatrixMode(GL_MODELVIEW);
	cDrawUtil::PushMatrix();
	mShadowCam.SetupGLView();

	DrawCharacters();
	DrawObjs();
	DrawMisc();
	glCullFace(GL_BACK);
	DrawGround();

	mShaderDepth->Unbind();
	mShadowMap->UnbindBuffer();

	glMatrixMode(GL_PROJECTION);
	cDrawUtil::PopMatrix();

	glMatrixMode(GL_MODELVIEW);
	cDrawUtil::PopMatrix();
}

void cDrawScenarioTerrainRL::DoDepthPass()
{
	mDepthTex->BindBuffer();
	mShaderDepth->Bind();

	glCullFace(GL_FRONT);

	cDrawUtil::ClearColor(tVector(1, 1, 1, 0));
	cDrawUtil::ClearDepth(1);

	DrawGround();
	DrawCharacters();
	DrawObjs();

	mShaderDepth->Unbind();
	mDepthTex->UnbindBuffer();

	glCullFace(GL_BACK);
}

void cDrawScenarioTerrainRL::DoAmbientOcclusion()
{
	GLboolean depth_state;
	glGetBooleanv(GL_DEPTH_TEST, &depth_state);
	glDisable(GL_DEPTH_TEST);

	mAOTex->BindBuffer();
	mShaderHBAO->Bind();
	mDepthTex->BindTex(GL_TEXTURE0);

	double near_z = mCam.GetNearZ();
	double far_z = mCam.GetFarZ();
	double fov = mCam.CalcFOV();
	double w = static_cast<double>(mAOTex->GetWidth());
	double h = static_cast<double>(mAOTex->GetHeight());

	tVector FocalLenLinMAD;
	tVector UVToViewAB;
	tVector AORes = tVector(w, h, 0, 0);

	FocalLenLinMAD[0] = 1.0 / tan(fov * 0.5) * (h / w);
	FocalLenLinMAD[1] = 1.0 / tan(fov * 0.5);
	FocalLenLinMAD[2] = (near_z - far_z) / (2.0 * near_z * far_z);
	FocalLenLinMAD[3] = (near_z + far_z) / (2.0 * near_z * far_z);

	UVToViewAB[0] = -2.0 / FocalLenLinMAD[0];
	UVToViewAB[1] = -2.0 / FocalLenLinMAD[1];
	UVToViewAB[2] = 1.0 / FocalLenLinMAD[0];
	UVToViewAB[3] = 1.0 / FocalLenLinMAD[1];

	mShaderHBAO->SetUniform4(mHBAOFocalLenLinMADHandle, FocalLenLinMAD);
	mShaderHBAO->SetUniform4(mHBAOUVToViewABHandle, UVToViewAB);
	mShaderHBAO->SetUniform2(mHBAOTexResHandle, AORes);

	// draw full screen quad
	cDrawUtil::DrawRect(tVector::Zero(), tVector(2, 2, 0, 0));

	if (depth_state == GL_TRUE)
	{
		glEnable(GL_DEPTH_TEST);
	}

	mDepthTex->UnbindTex(GL_TEXTURE0);
	mShaderHBAO->Unbind();
	mAOTex->UnbindBuffer();
}

void cDrawScenarioTerrainRL::DoGeoPass()
{
	mGBuffer->BindBuffer();
	mShaderGBuffer->Bind();

	cDrawUtil::ClearColor(tVector(1, 1, 1, 0));
	cDrawUtil::ClearDepth(1);

	DrawGround();
	DrawCharacters();
	DrawObjs();

	mShaderGBuffer->Unbind();
	mGBuffer->UnbindBuffer();
}

void cDrawScenarioTerrainRL::DoReflectionPass()
{
	GLboolean depth_state;
	glGetBooleanv(GL_DEPTH_TEST, &depth_state);
	glDisable(GL_DEPTH_TEST);

	mReflectionTex->BindBuffer();
	mShaderSSR->Bind();
	mGBuffer->BindTex(GL_TEXTURE0, cGBuffer::eGBufferTexDepth);
	mGBuffer->BindTex(GL_TEXTURE1, cGBuffer::eGBufferTexNormal);
	mIntBufferTex->BindTex(GL_TEXTURE2);
	//mDepthTex->BindTex(GL_TEXTURE3);

	double near_z = mCam.GetNearZ();
	double far_z = mCam.GetFarZ();
	double fov = mCam.CalcFOV();
	double w = static_cast<double>(mGBuffer->GetWidth());
	double h = static_cast<double>(mGBuffer->GetHeight());

	tVector FocalLenLinMAD;
	tVector UVToViewAB;
	tVector TexRes = tVector(w, h, 0, 0);

	FocalLenLinMAD[0] = 1.0 / tan(fov * 0.5) * (h / w);
	FocalLenLinMAD[1] = 1.0 / tan(fov * 0.5);
	FocalLenLinMAD[2] = (near_z - far_z) / (2.0 * near_z * far_z);
	FocalLenLinMAD[3] = (near_z + far_z) / (2.0 * near_z * far_z);

	UVToViewAB[0] = -2.0 / FocalLenLinMAD[0];
	UVToViewAB[1] = -2.0 / FocalLenLinMAD[1];
	UVToViewAB[2] = 1.0 / FocalLenLinMAD[0];
	UVToViewAB[3] = 1.0 / FocalLenLinMAD[1];

	mShaderSSR->SetUniform4(mSSRFocalLenLinMADHandle, FocalLenLinMAD);
	mShaderSSR->SetUniform4(mSSRUVToViewABHandle, UVToViewAB);
	mShaderSSR->SetUniform2(mSSRTexResHandle, TexRes);

	// draw full screen quad
	cDrawUtil::DrawRect(tVector::Zero(), tVector(2, 2, 0, 0));

	if (depth_state == GL_TRUE)
	{
		glEnable(GL_DEPTH_TEST);
	}

	//mDepthTex->UnbindTex(GL_TEXTURE3);
	mIntBufferTex->UnbindTex(GL_TEXTURE2);
	mGBuffer->UnbindTex(GL_TEXTURE1);
	mGBuffer->UnbindTex(GL_TEXTURE0);
	mShaderSSR->Unbind();
	mReflectionTex->UnbindBuffer();

	mPostProcessor->BlurTexture(*mReflectionTex, *mReflectionTex1, *mReflectionTex, 1.f);
	ApplyReflection(*mReflectionTex);
}

void cDrawScenarioTerrainRL::ApplyReflection(const cTextureDesc& ref_tex)
{
	GLboolean depth_state;
	glGetBooleanv(GL_DEPTH_TEST, &depth_state);
	glDisable(GL_DEPTH_TEST);

	mIntBufferTex->BindBuffer();
	glBlendFunc(GL_ONE, GL_ONE);
	
	mShaderApplySSR->Bind();

	ref_tex.BindTex(GL_TEXTURE0);

	cDrawUtil::DrawRect(tVector::Zero(), tVector(2, 2, 0, 0));

	mShaderApplySSR->Unbind();

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	mIntBufferTex->UnbindBuffer();

	if (depth_state == GL_TRUE)
	{
		glEnable(GL_DEPTH_TEST);
	}
}
