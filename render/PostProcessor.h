#pragma once
#include <stdlib.h>
#include <GL/glew.h>
#include <memory>

#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif
#include <fstream>

#include "Shader.h"
#include "TextureDesc.h"

class cPostProcessor
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cPostProcessor();
	~cPostProcessor(void);

	void Init(int window_w, int window_h);
	void DoPostProcessing(const cTextureDesc& input_frame, const cTextureDesc& output_frame) const;
	void SetSunScreenPos(const tVector& sun_pos);
	void Reshape(int w, int h);

	void BlurTexture(const cTextureDesc& input, const cTextureDesc& intermediate, 
				const cTextureDesc& output, float offset) const;

	virtual void setShaderRelativePath(std::string path);

private:
	static const int NUM_BLUR_WEIGHTS = 8;
	
	void LoadShaders();
	void LuminancePass(const cTextureDesc& input) const;
	void CrepuscularRay(const cTextureDesc& input, const cTextureDesc& intermediate,
						const cTextureDesc& output) const;
	void BloomPass(const cTextureDesc& input,
					const cTextureDesc& tex0, const cTextureDesc& tex1,
					const cTextureDesc& tex2, const cTextureDesc& tex3,
					const cTextureDesc& tex4, const cTextureDesc& tex5,
					const cTextureDesc& tex6, const cTextureDesc& tex7,
					const cTextureDesc& tex8, const cTextureDesc& tex9, 
					const cTextureDesc& output) const;
	void BloomComposite(const cTextureDesc& input0, const cTextureDesc& input1,
						const cTextureDesc& input2, const cTextureDesc& input3,
						const cTextureDesc& input4, const cTextureDesc& output) const;
	void BrightPass(const cTextureDesc& input, const cTextureDesc& lum_tex, 
					const cTextureDesc& output) const;
	void Blur(const cTextureDesc& input, const cTextureDesc& intermediate, 
				const cTextureDesc& output, float offset) const;
	void BlurPass(const cTextureDesc& input, const cTextureDesc& output,
					float offset_x, float offset_y) const;
	void FinalComposite(const cTextureDesc& base_tex, const cTextureDesc& bloom_tex,
						const cTextureDesc& crepuscular_tex, const cTextureDesc& output) const;
	void FXAA(const cTextureDesc& input, const cTextureDesc& output) const;

	void DrawPostFX(const cTextureDesc** const input_tex, int num_tex, const cTextureDesc& output, const cShader& shader) const;

	void CalcGaussianDistribution(float* weights, int num_weights) const;

	float mBlurWeights[NUM_BLUR_WEIGHTS];
	tVector mSunPos;

	// post processing resources
	// intermediate frame buffers
	std::unique_ptr<cTextureDesc> mFullRGBA;
	std::unique_ptr<cTextureDesc> mRGBA_1_HDR0;
	std::unique_ptr<cTextureDesc> mRGBA_1_HDR1;
	std::unique_ptr<cTextureDesc> mRGBA_2_HDR0;
	std::unique_ptr<cTextureDesc> mRGBA_2_HDR1;
	std::unique_ptr<cTextureDesc> mRGBA_4_HDR0;
	std::unique_ptr<cTextureDesc> mRGBA_4_HDR1;
	std::unique_ptr<cTextureDesc> mRGBA_8_HDR0;
	std::unique_ptr<cTextureDesc> mRGBA_8_HDR1;
	std::unique_ptr<cTextureDesc> mRGBA_16_HDR0;
	std::unique_ptr<cTextureDesc> mRGBA_16_HDR1;

	std::unique_ptr<cTextureDesc> mLumTex;

	// shader programs
	cShader mFXAAProg;
	cShader mLumProg;
	cShader mDownSampleProg;
	cShader mBrightProg;
	cShader mFinalCompProg;
	cShader mBlurProg;
	cShader mBloomCompositeProg;
	cShader mCrepuscularProg;

	// shader params
	GLuint mBlurWeightsHandle;
	GLuint mBlurOffsetHandle;
	GLuint mCrepuscularSunHandle;
	GLuint mCrepuscularLightHandle;

	std::string mRelativePath;
};

