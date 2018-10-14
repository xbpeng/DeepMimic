#include "ShadowMap.h"

cShadowMap::cShadowMap(void)
{
}

bool cShadowMap::Init(unsigned int width, unsigned int height)
{
	mWidth = width;
	mHeight = height;

	glEnable(GL_TEXTURE_2D);

	// depth buffer
	glGenTextures(1, &mTexture);
	glBindTexture(GL_TEXTURE_2D, mTexture);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);

	float border_color[4] = { 1.f, 1.f, 1.f, 1.f };
	glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border_color);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT16, width, height,
		0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);

	// linked depth stencil and render texture
	glGenFramebuffers(1, &mObject);
	glBindFramebuffer(GL_FRAMEBUFFER, mObject);

	glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, mTexture, 0);

	//Intel OpenGL driver crashes when using GL_NONE for glDrawBuffer on Linux, so use a workaround
#ifdef _WIN32
	glDrawBuffer(GL_NONE);
#else
	GLenum drawBuffers[2] = { GL_COLOR_ATTACHMENT0,0 };
	glDrawBuffers(1, drawBuffers);
#endif

	GLenum status;
	if ((status = glCheckFramebufferStatus(GL_FRAMEBUFFER)) != GL_FRAMEBUFFER_COMPLETE) {
		printf("texture is incomplete/n");
	}

	glBindTexture(GL_TEXTURE_2D, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	return true;
}

void cShadowMap::Reshape(int w, int h)
{
	mWidth = std::max(1, w);
	mHeight = std::max(1, h);
	if (mObject != 0) // 0 indicates the device's frame buffer, so no need to resize it
	{
		cTextureUtil::DeleteFrameBuffer(mObject, mTexture, mDepthStencil);
		Init(w, h);
	}
}

cShadowMap::~cShadowMap(void)
{
}
