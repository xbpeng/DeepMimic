#include "TextureDesc.h"
#include <assert.h>
#include "lodepng/lodepng.h"

std::stack<cTextureDesc::tTexEntry> cTextureDesc::gTexStack = std::stack<cTextureDesc::tTexEntry>();

cTextureDesc::tTexEntry::tTexEntry()
{
	mTex = 0;
	for (int i = 0; i < gNumViewportParams; ++i)
	{
		mViewportParams[i] = 0;
	}
}

cTextureDesc::cTextureDesc()
{
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

cTextureDesc::cTextureDesc( int width, int height, int channels, GLenum format, 
						   GLenum type, bool mipmaps )
	: mWidth( width ),
		mHeight( height ),
		mDepth(1),
		mChannels( channels ),
		mFormat( format ),
		mType( type ),
		mHasMips( mipmaps )
{
	cTextureUtil::CreateFrameBuffer(mObject, mTexture, mDepthStencil, mWidth, mHeight, mDepth,
									mChannels, mFormat, mType, mHasMips);
}

cTextureDesc::cTextureDesc( int width, int height, int depth, int channels, 
						   GLenum format, GLenum type, bool mipmaps )
	: mWidth( width ),
	  mHeight( height ),
	  mDepth(depth),
	  mChannels( channels ),
	  mFormat( format ),
	  mType( type ),
	  mHasMips( mipmaps )
{
	cTextureUtil::CreateFrameBuffer(mObject, mTexture, mDepthStencil, mWidth, mHeight, mDepth,
									mChannels, mFormat, mType, mHasMips);
}

cTextureDesc::cTextureDesc( GLuint obj, GLuint tex, GLuint ds, int width, int height, 
						    int depth, int channels, GLenum format )
	: mWidth( width ),
	  mHeight( height ),
	  mDepth(depth),
	  mChannels( channels ),
	  mFormat( format ),
	  mObject( obj ),
	  mTexture( tex ),
	  mDepthStencil( ds )
{
}

cTextureDesc::cTextureDesc(const std::string& filename, bool gen_mips) : cTextureDesc()
{
	std::vector<unsigned char> image;
	unsigned width, height;
	unsigned error = lodepng::decode(image, width, height, filename);

	if (error == 0)
	{
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

		if (gen_mips)
		{
			glGenerateMipmap(GL_TEXTURE_2D);
			
			GLfloat max_aniso = 1.f;
			glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &max_aniso);
			glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, max_aniso);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		}
		else
		{
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST_MIPMAP_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
		}

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

		glBindTexture(GL_TEXTURE_2D, 0);
	}
}

void cTextureDesc::BindBuffer() const
{
	if (IsRenderBuffer())
	{
		glBindFramebuffer(GL_FRAMEBUFFER, mObject);
		glViewport(0, 0, mWidth, mHeight);
		PushTextureStack();
	}
}

void cTextureDesc::UnbindBuffer() const
{
	if (IsRenderBuffer())
	{
		PopTextureStack();
	}
}

bool cTextureDesc::CheckBoundBuffer() const
{
	GLint curr_obj;
	glGetIntegerv(GL_FRAMEBUFFER_BINDING, &curr_obj);
	return curr_obj == mObject;
}

void cTextureDesc::BindBuffer3DSlice(int slice) const
{
	glBindFramebuffer(GL_FRAMEBUFFER, mObject);
	glFramebufferTexture3DEXT(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, 
							GL_TEXTURE_3D, mObject, 0, slice);
	glViewport(0, 0, mWidth, mHeight);
	PushTextureStack();
}

void cTextureDesc::BindTex(GLint tex_slot) const
{
	glActiveTexture(tex_slot);
	glBindTexture(GL_TEXTURE_2D, GetTexture());
}

void cTextureDesc::UnbindTex(GLint tex_slot) const
{
	glActiveTexture(tex_slot);
	glBindTexture(GL_TEXTURE_2D, 0);
}

GLuint cTextureDesc::GetObj() const
{
	return mObject;
}

GLuint cTextureDesc::GetTexture() const
{
	return mTexture;
}

GLuint cTextureDesc::GetDepthStencil() const
{
	return mDepthStencil;
}

bool cTextureDesc::IsValid() const
{
	return mTexture != -1;
}

bool cTextureDesc::IsRenderBuffer() const
{
	return mObject != -1;
}

int cTextureDesc::GetWidth() const
{
	return mWidth;
}

int cTextureDesc::GetHeight() const
{
	return mHeight;
}

int cTextureDesc::GetNumTexels() const
{
	return GetWidth() * GetHeight();
}

int cTextureDesc::GetNumChannels() const
{
	int channels = cTextureUtil::GetNumChannels(mChannels);
	return channels;
}

void cTextureDesc::Reshape( int w, int h )
{
	mWidth = std::max(1, w);
	mHeight = std::max(1, h);
	if (mObject != 0) // 0 indicates the device's frame buffer, so no need to resize it
	{
		cTextureUtil::DeleteFrameBuffer(mObject, mTexture, mDepthStencil);
		cTextureUtil::CreateFrameBuffer(mObject, mTexture, mDepthStencil, mWidth, mHeight,
							mDepth, mChannels, mFormat, mType, mHasMips);
	}
}

void cTextureDesc::ReadPixels(std::vector<GLfloat>& out_data, GLenum format/*= GL_RGBA*/)
{
	BindBuffer();

	int num_channels = GetNumChannels();
	int data_size = GetNumTexels() * num_channels;
	out_data.resize(data_size);
	glReadPixels(0, 0, mWidth, mHeight, format, GL_FLOAT, out_data.data());

	UnbindBuffer();
}

void cTextureDesc::WritePixels(const std::vector<GLubyte>& data)
{
	glBindTexture(GL_TEXTURE_2D, mTexture);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, mWidth, mHeight, mFormat, mType, data.data());
	glBindTexture(GL_TEXTURE_2D, 0);
}

cTextureDesc::~cTextureDesc()
{
	cTextureUtil::DeleteFrameBuffer( mObject, mTexture, mDepthStencil );
}

void cTextureDesc::PushTextureStack() const
{
	tTexEntry entry;
	entry.mTex = mObject;
	entry.mViewportParams[0] = 0;
	entry.mViewportParams[1] = 0;
	entry.mViewportParams[2] = mWidth;
	entry.mViewportParams[3] = mHeight;

	gTexStack.push(entry);
}

void cTextureDesc::PopTextureStack() const
{
	bool bound = CheckBoundBuffer();
	if (bound)
	{
		gTexStack.pop();
		if (!gTexStack.empty())
		{
			tTexEntry prev_entry = gTexStack.top();
			glBindFramebuffer(GL_FRAMEBUFFER, prev_entry.mTex);
			glViewport(prev_entry.mViewportParams[0], prev_entry.mViewportParams[1],
				prev_entry.mViewportParams[2], prev_entry.mViewportParams[3]);
		}
	}
	else
	{
		throw std::runtime_error("Trying to pop unbound texture from stack.\n");
	}
}
