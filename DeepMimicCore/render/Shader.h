#pragma once

#include<stack>
#include <stdlib.h>
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <fstream>

#include "util/MathUtil.h"

class cShader
{
public:
	enum eUniform
	{
		eUniformModelViewMatrix,
		eUniformProjectionMatrix,
		eUniformColor,
		eUniformMax
	};

	static const cShader* GetBoundShader();

	cShader(void);
	~cShader(void);

	virtual void Clear();
	virtual bool BuildShader(const std::string& vs_filename, const std::string& ps_filename);
	virtual void GetAttributeHandle(GLuint& attribute_handle, const std::string& attribute_name);
	virtual void GetUniformHandle(GLuint& uniform_handle, const std::string& uniform_name, bool check_valid = false);

	virtual void Bind() const;
	virtual void Unbind() const;

	virtual void SetUniform2(GLuint handle, const tVector& data) const;
	virtual void SetUniform3(GLuint handle, const tVector& data) const;
	virtual void SetUniform4(GLuint handle, const tVector& data) const;
	virtual void SetUniformMat(GLuint handle, const tMatrix& data) const;

	virtual bool HasUniform(eUniform unif) const;
	virtual void SetUniform4(eUniform unif, const tVector& data) const;
	virtual void SetUniformMat(eUniform unif, const tMatrix& data) const;

	virtual bool CheckBoundShader() const;
	virtual GLuint GetProg() const;


protected:
	struct tShaderEntry
	{
		const cShader* mShader;
		tShaderEntry();
	};

	static std::stack<tShaderEntry> gShaderStack;

	GLuint mProg;
	GLuint mUniformHandles[eUniformMax];

	virtual GLuint LoadShader(const std::string& filename, GLenum shader_type);
	virtual GLuint CreateShader(GLuint& vsh_handle, GLuint& psh_handle);
	virtual void RecordUniformHandles();

	virtual void PushShaderStack() const;
	virtual void PopShaderStack() const;
};
