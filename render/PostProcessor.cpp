#include "PostProcessor.h"
#include <iostream>
#include <cmath>
#include "render/Shader.h"
#include "render/TextureUtil.h"
#include "render/DrawUtil.h"

cPostProcessor::cPostProcessor()
{
	mSunPos[0] = 0.f;
	mSunPos[1] = 0.f;
	mSunPos[2] = 0.f;
	mRelativePath = "";
}

void cPostProcessor::Init(int window_w, int window_h)
{
	// create buffers
	mFullRGBA = std::unique_ptr<cTextureDesc>(new cTextureDesc(window_w, window_h, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, false));

	mRGBA_1_HDR0 = std::unique_ptr<cTextureDesc>(new cTextureDesc(window_w, window_h, GL_RGBA16F, GL_RGBA, GL_FLOAT, false));
	mRGBA_1_HDR1 = std::unique_ptr<cTextureDesc>(new cTextureDesc(window_w, window_h, GL_RGBA16F, GL_RGBA, GL_FLOAT, false));
	
	mRGBA_2_HDR0 = std::unique_ptr<cTextureDesc>(new cTextureDesc(window_w / 2, window_h / 2, GL_RGBA16F, GL_RGBA, GL_FLOAT, false));
	mRGBA_2_HDR1 = std::unique_ptr<cTextureDesc>(new cTextureDesc(window_w / 2, window_h / 2, GL_RGBA16F, GL_RGBA, GL_FLOAT, false));
	
	mRGBA_4_HDR0 = std::unique_ptr<cTextureDesc>(new cTextureDesc(window_w / 4, window_h / 4, GL_RGBA16F, GL_RGBA, GL_FLOAT, false));
	mRGBA_4_HDR1 = std::unique_ptr<cTextureDesc>(new cTextureDesc(window_w / 4, window_h / 4, GL_RGBA16F, GL_RGBA, GL_FLOAT, false));

	mRGBA_8_HDR0 = std::unique_ptr<cTextureDesc>(new cTextureDesc(window_w / 8, window_h / 8, GL_RGBA16F, GL_RGBA, GL_FLOAT, false));
	mRGBA_8_HDR1 = std::unique_ptr<cTextureDesc>(new cTextureDesc(window_w / 8, window_h / 8, GL_RGBA16F, GL_RGBA, GL_FLOAT, false));

	mRGBA_16_HDR0 = std::unique_ptr<cTextureDesc>(new cTextureDesc(window_w / 16, window_h / 16, GL_RGBA16F, GL_RGBA, GL_FLOAT, false));
	mRGBA_16_HDR1 = std::unique_ptr<cTextureDesc>(new cTextureDesc(window_w / 16, window_h / 16, GL_RGBA16F, GL_RGBA, GL_FLOAT, false));

	mLumTex = std::unique_ptr<cTextureDesc>(new cTextureDesc(1024, 1024,  GL_R32F, GL_RED, GL_FLOAT, true));

	// Luminance Pass
	{
		mLumProg.setShaderRelativePath(mRelativePath);
		std::cout << "*** cPostProcessor relative path: " << mRelativePath << std::endl;
		mLumProg.BuildShader("render/shaders/FullScreenQuad_VS.glsl", "render/shaders/Luminance_PS.glsl");
	}

	// Down Sample
	{
		mDownSampleProg.setShaderRelativePath(mRelativePath);
		mDownSampleProg.BuildShader("render/shaders/FullScreenQuad_VS.glsl", "render/shaders/DownSample_PS.glsl");
	}

	// Bright Pass
	{
		mBrightProg.setShaderRelativePath(mRelativePath);
		mBrightProg.BuildShader("render/shaders/FullScreenQuad_VS.glsl", "render/shaders/BrightPass_PS.glsl");
		
		mBrightProg.Bind();
		GLint lum_tex = glGetUniformLocation(mBrightProg.GetProg(), "gLumTex");
		glUniform1i(lum_tex, 1);
		mBrightProg.Unbind();
	}

	// Bloom Composite 
	{
		mBloomCompositeProg.setShaderRelativePath(mRelativePath);
		mBloomCompositeProg.BuildShader("render/shaders/FullScreenQuad_VS.glsl", "render/shaders/BloomComposite_PS.glsl");

		mBloomCompositeProg.Bind();
		GLint bloom_tex = glGetUniformLocation(mBloomCompositeProg.GetProg(), "gBloomTex0");
		glUniform1i(bloom_tex, 0);
		bloom_tex = glGetUniformLocation(mBloomCompositeProg.GetProg(), "gBloomTex1");
		glUniform1i(bloom_tex, 1);
		bloom_tex = glGetUniformLocation(mBloomCompositeProg.GetProg(), "gBloomTex2");
		glUniform1i(bloom_tex, 2);
		bloom_tex = glGetUniformLocation(mBloomCompositeProg.GetProg(), "gBloomTex3");
		glUniform1i(bloom_tex, 3);
		bloom_tex = glGetUniformLocation(mBloomCompositeProg.GetProg(), "gBloomTex4");
		glUniform1i(bloom_tex, 4);
		mBloomCompositeProg.Unbind();
	}

	// Blur
	{
		mBlurProg.setShaderRelativePath(mRelativePath);
		mBlurProg.BuildShader("render/shaders/FullScreenQuad_VS.glsl", "render/shaders/Blur_PS.glsl");

		mBlurProg.BindShaderUniform(mBlurWeightsHandle, "gWeights");
		mBlurProg.BindShaderUniform(mBlurOffsetHandle, "gOffset");
	}

	// Crepuscular
	{
		mCrepuscularProg.setShaderRelativePath(mRelativePath);
		mCrepuscularProg.BuildShader("render/shaders/FullScreenQuad_VS.glsl", "render/shaders/Crepuscular_PS.glsl");
		mCrepuscularProg.BindShaderUniform(mCrepuscularSunHandle, "gSunParams");
		mCrepuscularProg.BindShaderUniform(mCrepuscularLightHandle, "gLightColour");
	}
	
	// Final Composite, also doing tonemapping here
	{
		mFinalCompProg.setShaderRelativePath(mRelativePath);
		mFinalCompProg.BuildShader("render/shaders/FullScreenQuad_VS.glsl", "render/shaders/FinalComposite_PS.glsl");
		
		// bind texture samplers
		GLint buffer_tex = glGetUniformLocation(mFinalCompProg.GetProg(), "gBufferTex");
		GLint lum_tex = glGetUniformLocation(mFinalCompProg.GetProg(), "gLumTex");
		GLint bloom_tex = glGetUniformLocation(mFinalCompProg.GetProg(), "gBloomTex");
		GLint crepuscular_tex = glGetUniformLocation(mFinalCompProg.GetProg(), "gCrepuscularTex");
	
		mFinalCompProg.Bind();
		glUniform1i(buffer_tex, 0);
		glUniform1i(lum_tex, 1);
		glUniform1i(bloom_tex, 2);
		glUniform1i(crepuscular_tex, 3);
		mFinalCompProg.Unbind();
	}

	// FXAA
	{
		mFXAAProg.setShaderRelativePath(mRelativePath);
		mFXAAProg.BuildShader("render/shaders/FullScreenQuad_VS.glsl", "render/shaders/FXAA_PS.glsl");
	}

	CalcGaussianDistribution(mBlurWeights, NUM_BLUR_WEIGHTS);
}

void cPostProcessor::LoadShaders()
{
}

void cPostProcessor::SetSunScreenPos(const tVector& sun_pos)
{
	mSunPos = sun_pos;
}

void cPostProcessor::Reshape(int w, int h)
{
	mFullRGBA->Reshape(w, h);

	mRGBA_1_HDR0->Reshape(w, h);
	mRGBA_1_HDR1->Reshape(w, h);
	
	mRGBA_2_HDR0->Reshape(w / 2, h / 2);
	mRGBA_2_HDR1->Reshape(w / 2, h / 2);
	
	mRGBA_4_HDR0->Reshape(w / 4, h / 4);
	mRGBA_4_HDR1->Reshape(w / 4, h / 4);

	mRGBA_8_HDR0->Reshape(w / 8, h / 8);
	mRGBA_8_HDR1->Reshape(w / 8, h / 8);

	mRGBA_16_HDR0->Reshape(w / 16, h / 16);
	mRGBA_16_HDR1->Reshape(w / 16, h / 16);
}

void cPostProcessor::DoPostProcessing(const cTextureDesc& input, const cTextureDesc& output) const
{
	// store previous matrices
	glMatrixMode(GL_PROJECTION);
	cDrawUtil::PushMatrix();
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
	cDrawUtil::PushMatrix();
    glLoadIdentity();

	glDisable(GL_BLEND);
	LuminancePass(input);

	cTextureDesc& bloom_tex = *mRGBA_1_HDR1;
	BloomPass(input,
				*mRGBA_1_HDR0, *mRGBA_1_HDR1,
				*mRGBA_2_HDR0, *mRGBA_2_HDR1,
				*mRGBA_4_HDR0, *mRGBA_4_HDR1,
				*mRGBA_8_HDR0, *mRGBA_8_HDR1,
				*mRGBA_16_HDR0, *mRGBA_16_HDR1,
				bloom_tex);

	const cTextureDesc& crepuscular_tex = *mRGBA_2_HDR0;
	CrepuscularRay(input, *mRGBA_2_HDR1, crepuscular_tex);

	FinalComposite(input, bloom_tex, crepuscular_tex, *mFullRGBA);
	FXAA(*mFullRGBA, output);
	//FXAA(input, output);
	glEnable(GL_BLEND);

	// restore previous matrices
	glMatrixMode(GL_PROJECTION);
	cDrawUtil::PopMatrix();
    glMatrixMode(GL_MODELVIEW);
	cDrawUtil::PopMatrix();
}

void cPostProcessor::DrawPostFX(const cTextureDesc** const input_tex, int num_tex, const cTextureDesc& output, const cShader& shader) const
{
	GLboolean depth_state;
	glGetBooleanv(GL_DEPTH_TEST, &depth_state);
	glDisable(GL_DEPTH_TEST);

	output.BindBuffer();

	shader.Bind();
	// for now just bind the first entry of the inputs

	for (int i = 0; i < num_tex; ++i)
	{
		input_tex[i]->BindTex(GL_TEXTURE0 + i);
	}

	// draw full screen quad
	cDrawUtil::DrawRect(tVector::Zero(), tVector(2, 2, 0, 0));

	shader.Unbind();

	for (int i = 0; i < num_tex; ++i)
	{
		input_tex[i]->UnbindTex(GL_TEXTURE0 + i);
	}

	if (depth_state == GL_TRUE)
	{
		glEnable(GL_DEPTH_TEST);
	}

	output.UnbindBuffer();
}

void cPostProcessor::LuminancePass(const cTextureDesc& input) const
{
	int num_inputs = 1;
	const cTextureDesc* input_text[] = { &input };
	DrawPostFX(input_text, num_inputs, *mLumTex, mLumProg);

	// generate mipmaps
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, mLumTex->GetTexture());
	glGenerateMipmap(GL_TEXTURE_2D);
}

void cPostProcessor::CrepuscularRay(const cTextureDesc& input, const cTextureDesc& intermediate, 
									const cTextureDesc& output) const
{
	int num_inputs = 1;
	const cTextureDesc* input_text[] = { &input };

	float sun_params[] = { static_cast<float>(mSunPos[0]), static_cast<float>(mSunPos[1]), 
							static_cast<float>(mSunPos[2]), 1.f };
	glProgramUniform4fv(mCrepuscularProg.GetProg(), mCrepuscularSunHandle, 1, sun_params);

	float light_colour[] = { 0.3f, 0.24f, 0.15f, 0.2f };
	glProgramUniform4fv(mCrepuscularProg.GetProg(), mCrepuscularLightHandle, 1, light_colour);

	DrawPostFX(input_text, num_inputs, output, mCrepuscularProg);
	Blur(output, intermediate, output, 0.25f);
}

void cPostProcessor::BloomPass(const cTextureDesc& input,
								const cTextureDesc& tex0, const cTextureDesc& tex1,
								const cTextureDesc& tex2, const cTextureDesc& tex3,
								const cTextureDesc& tex4, const cTextureDesc& tex5,
								const cTextureDesc& tex6, const cTextureDesc& tex7,
								const cTextureDesc& tex8, const cTextureDesc& tex9, 
								const cTextureDesc& output) const
{
	// bright pass to extract bright spots
	BrightPass(input, *mLumTex, tex0);
	// pass 1
	Blur(tex0, tex1, tex0, 1.f); // single texel blur
	// pass 2
	Blur(tex0, tex3, tex2, 1.f);
	// pass 3
	Blur(tex2, tex5, tex4, 1.f);
	// pass 4
	Blur(tex4, tex7, tex6, 1.f);
	// pass 5
	Blur(tex6, tex9, tex8, 1.f);

	// composite
	BloomComposite(tex0, tex2, tex4, tex6, tex8, output);
}

void cPostProcessor::BrightPass(const cTextureDesc& input, const cTextureDesc& lum_tex, const cTextureDesc& output) const
{
	int num_inputs = 2;
	const cTextureDesc* input_text[] = { &input, &lum_tex };
	// downsample the input 5 times
	// first pass is a bright pass that extracts all bright spots
	DrawPostFX(input_text, 2, output, mBrightProg);
}

void cPostProcessor::BloomComposite(const cTextureDesc& input0,
									const cTextureDesc& input1,
									const cTextureDesc& input2,
									const cTextureDesc& input3,
									const cTextureDesc& input4,
									const cTextureDesc& output) const
{
	int num_inputs = 5;
	const cTextureDesc* input_text[] = { &input0, &input1, &input2, 
										&input3, &input4 };
	DrawPostFX(input_text, num_inputs, output, mBloomCompositeProg);
}

// for external use
void cPostProcessor::BlurTexture(const cTextureDesc& input, const cTextureDesc& intermediate, 
						  const cTextureDesc& output, float offset) const
{
	GLboolean blend_state;
	glGetBooleanv(GL_BLEND, &blend_state);
	glDisable(GL_BLEND);

	Blur(input, intermediate, output, offset);
	
	if (blend_state == 1)
	{
		glEnable(GL_BLEND);
	}
}


void cPostProcessor::Blur(const cTextureDesc& input, const cTextureDesc& intermediate, 
						  const cTextureDesc& output, float offset) const
{
	BlurPass(input, intermediate, offset, 0.f);
	BlurPass(intermediate, output, 0.f, offset);
}

void cPostProcessor::BlurPass(const cTextureDesc& input, const cTextureDesc& output, 
								float offset_x, float offset_y) const
{
	const cTextureDesc* input_tex[] = {&input};

	float offset[] = {offset_x / input.GetWidth(), offset_y / input.GetHeight(), 0.f, 0.f}; // single texel offset

	// offset the initial tap by half a pixel so we don't over sample the center tap
	if (offset_x != 0.f)
	{
		offset[2] = 0.5f / input.GetWidth();
	}
	if (offset_y != 0.f)
	{
		offset[3] = 0.5f / input.GetHeight();
	}

	glProgramUniform1fv(mBlurProg.GetProg(), mBlurWeightsHandle, NUM_BLUR_WEIGHTS, mBlurWeights);
	glProgramUniform4fv(mBlurProg.GetProg(), mBlurOffsetHandle, 1, offset);

	DrawPostFX(input_tex, 1, output, mBlurProg);
}

void cPostProcessor::FinalComposite(const cTextureDesc& base_tex, const cTextureDesc& bloom_tex, 
									const cTextureDesc& crepuscular_tex, const cTextureDesc& output) const
{
	int num_inputs = 4;
	const cTextureDesc* input_text[] = { &base_tex, mLumTex.get(), &bloom_tex, &crepuscular_tex };
	DrawPostFX(input_text, num_inputs, output, mFinalCompProg);
}

void cPostProcessor::FXAA(const cTextureDesc& input, const cTextureDesc& output) const
{
	int num_inputs = 1;
	const cTextureDesc* input_text[] = { &input };
	DrawPostFX(input_text, num_inputs, output, mFXAAProg);
}

void cPostProcessor::CalcGaussianDistribution(float* weights, int num_weights) const
{
	float x = 0.f;
	float std_dev = 0.5f;
	float offset = 3 * std_dev / num_weights;
	// the weights returned are always from 0 to 2 standard deviations
	// they are only on one side of the kernel
	// the width of the samples also range from 0 to 1
	float sum = 0.f;
	for (int i = 0; i < num_weights; ++i)
	{
		x = i * offset;
		weights[ i ] = static_cast<float>(std::exp(-x * x) / (2 * std_dev * std_dev) / (std_dev * std::sqrt(2 * M_PI)));
		sum += weights[ i ];
	}

	// normalize the weights
	// double the sum since these weights are only half the distribution
	sum *= 2.f;
	for (int i = 0; i < num_weights; ++i)
	{
		weights[i] /= sum;
	}
}

cPostProcessor::~cPostProcessor(void)
{
}

void cPostProcessor::setShaderRelativePath(std::string path)
{
	this->mRelativePath = path;
}
