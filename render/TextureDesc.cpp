#include "TextureDesc.h"
#include "LodePNG/lodepng.h"
#include <assert.h>

std::stack<cTextureDesc::tTexEntry> cTextureDesc::mTexStack = std::stack<cTextureDesc::tTexEntry>();
cTextureDesc::tTexEntry::tTexEntry() {
    mTex = 0;
    for (int i = 0; i < gNumViewportParams; ++i) {
        mViewportParams[i] = 0;
    }
}

cTextureDesc::cTextureDesc() {
    mWidth = 0;
    mHeight = 0;
    mDepth = 0;
    mChannels = GL_RGBA8;
    mType = GL_UNSIGNED_BYTE;
    mFormat = GL_RGBA;
    mHasMips = false;
    mObject = -1;
    mTexture = -1;
    mDepthStencil = -1;
}

cTextureDesc::cTextureDesc(int width, int height, int channels, GLenum format, GLenum type, bool mipmaps)
    : mWidth(width), mHeight(height), mDepth(1), mChannels(channels), mFormat(format), mType(type), mHasMips(mipmaps) {
    CreateFrameBuffer(mObject, mTexture, mDepthStencil, mWidth, mHeight, mDepth, mChannels, mFormat, mType, mHasMips);
}

cTextureDesc::cTextureDesc(int width, int height, int depth, int channels, GLenum format, GLenum type, bool mipmaps)
    : mWidth(width), mHeight(height), mDepth(depth), mChannels(channels), mFormat(format), mType(type),
      mHasMips(mipmaps) {
    CreateFrameBuffer(mObject, mTexture, mDepthStencil, mWidth, mHeight, mDepth, mChannels, mFormat, mType, mHasMips);
}

cTextureDesc::cTextureDesc(GLuint obj, GLuint tex, GLuint ds, int width, int height, int depth, int channels,
                           GLenum format)
    : mWidth(width), mHeight(height), mDepth(depth), mChannels(channels), mFormat(format), mObject(obj), mTexture(tex),
      mDepthStencil(ds) {}

cTextureDesc::cTextureDesc(const std::string &filename, bool gen_mips) : cTextureDesc() {
    std::vector<unsigned char> image;
    unsigned width, height;
    unsigned error = lodepng::decode(image, width, height, filename);

    if (error == 0) {
        mWidth = width;
        mHeight = height;
        mDepth = 1;
        mChannels = GL_RGBA8;
        mType = GL_UNSIGNED_BYTE;
        mFormat = GL_RGBA;
        mHasMips = true;
        int num_mipmaps = (gen_mips) ? static_cast<int>(log2(std::min(mWidth, mHeight))) : 1;

        glGenTextures(1, &mTexture);
        glBindTexture(GL_TEXTURE_2D, mTexture);
        glTexStorage2D(GL_TEXTURE_2D, num_mipmaps, mChannels, mWidth, mHeight);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, mWidth, mHeight, mFormat, mType, image.data());

        if (gen_mips) {
            glGenerateMipmap(GL_TEXTURE_2D);

            GLfloat max_aniso = 1.f;
            glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &max_aniso);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, max_aniso);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        } else {
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST_MIPMAP_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
        }

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

        glBindTexture(GL_TEXTURE_2D, 0);
    }
}

void cTextureDesc::BindBuffer() const {
    if (IsRenderBuffer()) {
        RecordPrevParams();
        glBindFramebuffer(GL_FRAMEBUFFER, mObject);
        glViewport(0, 0, mWidth, mHeight);
    }
}

void cTextureDesc::UnbindBuffer() const {
    if (IsRenderBuffer()) {
        RestorePrevParams();
    }
}

void cTextureDesc::BindBuffer3DSlice(int slice) const {
    RecordPrevParams();
    glBindFramebuffer(GL_FRAMEBUFFER, mObject);
    glFramebufferTexture3DEXT(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_3D, mObject, 0, slice);
    glViewport(0, 0, mWidth, mHeight);
}

void cTextureDesc::BindTex(GLint tex_slot) const {
    glActiveTexture(tex_slot);
    glBindTexture(GL_TEXTURE_2D, GetTexture());
}

void cTextureDesc::UnbindTex(GLint tex_slot) const {
    glActiveTexture(tex_slot);
    glBindTexture(GL_TEXTURE_2D, 0);
}

GLuint cTextureDesc::GetObj() const { return mObject; }

GLuint cTextureDesc::GetTexture() const { return mTexture; }

GLuint cTextureDesc::GetDepthStencil() const { return mDepthStencil; }

bool cTextureDesc::IsValid() const { return mTexture != -1; }

bool cTextureDesc::IsRenderBuffer() const { return mObject != -1; }

int cTextureDesc::GetWidth() const { return mWidth; }

int cTextureDesc::GetHeight() const { return mHeight; }

int cTextureDesc::GetNumTexels() const { return GetWidth() * GetHeight(); }

void cTextureDesc::Reshape(int w, int h) {
    mWidth = std::max(1, w);
    mHeight = std::max(1, h);
    if (mObject != 0) // 0 indicates the device's frame buffer, so no need to resize it
    {
        DeleteFrameBuffer(mObject, mTexture, mDepthStencil);
        CreateFrameBuffer(mObject, mTexture, mDepthStencil, mWidth, mHeight, mDepth, mChannels, mFormat, mType,
                          mHasMips);
    }
}

void cTextureDesc::ReadPixels(std::vector<GLfloat> &out_data) {
    BindBuffer();

    int num_output_channels = 4;
    int data_size = GetNumTexels() * num_output_channels;
    out_data.resize(data_size);
    glReadPixels(0, 0, mWidth, mHeight, GL_RGBA, GL_FLOAT, out_data.data());

    UnbindBuffer();
}

cTextureDesc::~cTextureDesc() { DeleteFrameBuffer(mObject, mTexture, mDepthStencil); }

void cTextureDesc::RestorePrevParams() const {
    if (mTexStack.size() > 0) {
        GLint curr_obj;
        glGetIntegerv(GL_FRAMEBUFFER_BINDING, &curr_obj);
        if (curr_obj == mObject) {
            tTexEntry entry = mTexStack.top();
            mTexStack.pop();

            glBindFramebuffer(GL_FRAMEBUFFER, entry.mTex);
            glViewport(entry.mViewportParams[0], entry.mViewportParams[1], entry.mViewportParams[2],
                       entry.mViewportParams[3]);
        }
    }
}

void cTextureDesc::RecordPrevParams() const {
    tTexEntry entry;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &(entry.mTex));
    glGetIntegerv(GL_VIEWPORT, entry.mViewportParams);
    mTexStack.push(entry);
}
