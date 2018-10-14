#pragma once

#include <vector>
#include <stack>
#include <GL/glew.h>
#include <GL/freeglut.h>

#include "TextureUtil.h"

class cTextureDesc
{
public:
	cTextureDesc(int width, int height, int channels, GLenum format, 
				GLenum type, bool mipmaps);
	cTextureDesc(int width, int height, int depth, int channels, GLenum format, 
				GLenum type, bool mipmaps);
	cTextureDesc(GLuint obj, GLuint tex, GLuint ds, int width, int height, int depth, 
					int channels, GLenum format);
	cTextureDesc(const std::string& filename, bool gen_mips = false);
	virtual ~cTextureDesc();

	virtual void BindBuffer() const;
	virtual void UnbindBuffer() const;
	virtual bool CheckBoundBuffer() const;
	virtual void BindBuffer3DSlice(int slice) const;
	virtual void BindTex(GLint tex_slot) const;
	virtual void UnbindTex(GLint tex_slot) const;

	virtual GLuint GetObj() const;
	virtual GLuint GetTexture() const;
	virtual GLuint GetDepthStencil() const;
	virtual bool IsValid() const;
	virtual bool IsRenderBuffer() const;

	virtual int GetWidth() const;
	virtual int GetHeight() const;
	virtual int GetNumTexels() const;
	virtual int GetNumChannels() const;

	virtual void Reshape(int w, int h);
	virtual void ReadPixels(std::vector<GLfloat>& out_data, GLenum format=GL_RGBA);
	virtual void WritePixels(const std::vector<GLubyte>& data);

protected:
	static const int gNumViewportParams = 4;
	struct tTexEntry
	{
		GLint mTex;
		GLint mViewportParams[gNumViewportParams];
		tTexEntry();
	};
	static std::stack<tTexEntry> gTexStack;

	int mWidth;
	int mHeight;
	int mDepth;
	int mChannels;
	GLenum mFormat;
	GLenum mType;
	bool mHasMips;

	GLuint mObject;
	GLuint mTexture;
	GLuint mDepthStencil;

	cTextureDesc();
	virtual void PushTextureStack() const;
	virtual void PopTextureStack() const;
};