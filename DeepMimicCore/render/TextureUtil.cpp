#include "TextureUtil.h"
#include <iostream>

void cTextureUtil::CreateFrameBuffer(GLuint& buffer_obj, GLuint& texture, GLuint& depth_stencil,
	int width, int height, int depth, int channels, GLenum format,
	GLenum type, bool mipmaps)
{
	bool is_3d_tex = depth > 1;
	glGenTextures(1, &texture);
	GLenum tex_type = GL_TEXTURE_2D;
	if (is_3d_tex)
	{
		tex_type = GL_TEXTURE_3D;
	}

	glBindTexture(tex_type, texture);

	glTexParameteri(tex_type, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(tex_type, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(tex_type, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(tex_type, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	if (is_3d_tex)
	{
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	}

	// generate the top level
	if (is_3d_tex)
	{
		glTexImage3D(GL_TEXTURE_3D, 0, channels, width, height, depth, 0, format, type, 0);
	}
	else
	{
		glTexImage2D(GL_TEXTURE_2D, 0, channels, width, height, 0, format, type, 0);
	}

	// generate the mips
	if (mipmaps)
	{
		glTexParameteri(tex_type, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glEnable(tex_type);
		glGenerateMipmap(tex_type);
	}

	glBindTexture(tex_type, 0);

	// depth buffer
	glGenRenderbuffers(1, &depth_stencil);
	glBindRenderbuffer(GL_RENDERBUFFER, depth_stencil);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, width, height);
	glBindRenderbuffer(GL_RENDERBUFFER, 0);

	// linked depth stencil and render texture
	glGenFramebuffers(1, &buffer_obj);
	if (is_3d_tex)
	{
		glBindFramebuffer(GL_RENDERBUFFER, buffer_obj);
		glFramebufferTexture(GL_RENDERBUFFER, GL_COLOR_ATTACHMENT0, texture, 0);

	}
	else
	{
		glBindFramebuffer(GL_FRAMEBUFFER, buffer_obj);
#ifdef _LINUX_
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture, 0);
#else
		glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, texture, 0);
#endif
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_stencil);
	}

	GLenum status;
	GLenum z = GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER;
	if ((status = glCheckFramebufferStatus(GL_FRAMEBUFFER)) != GL_FRAMEBUFFER_COMPLETE) {
		if (is_3d_tex)
		{
			std::cout << "fail to create 3d texture\n";
		}
		else
		{
			std::cout << "texture is incomplete\n";
		}
	}

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void cTextureUtil::DeleteFrameBuffer(GLuint& buffer_obj, GLuint& texture, GLuint& depth_stencil)
{
	glDeleteRenderbuffersEXT(1, &depth_stencil);
	glDeleteTextures(1, &texture);
	glDeleteFramebuffersEXT(1, &buffer_obj);
}

tVector cTextureUtil::ReadTexel(int x, int y, int w, int h, const std::vector<GLfloat>& data)
{
	const int num_channels = 4;
	size_t idx = static_cast<size_t>((w * y + x) * num_channels);
	GLfloat r = data[idx];
	GLfloat g = data[idx + 1];
	GLfloat b = data[idx + 2];
	GLfloat a = data[idx + 3];

	tVector texel = tVector(r, g, b, a);
	return texel;
}

int cTextureUtil::GetNumChannels(GLint channels)
{
	int c = 0;
	switch (channels)
	{
	case GL_DEPTH_COMPONENT:
	case GL_RED:
		c = 1;
		break;
	case GL_DEPTH_STENCIL:
	case GL_RG:
		c = 2;
		break;
	case GL_RGB:
		c = 3;
		break;
	case GL_RGBA:
		c = 4;
		break;
	default:
		throw std::runtime_error("Unsupported channel format.\n");
		break;
	}
	return c;
}