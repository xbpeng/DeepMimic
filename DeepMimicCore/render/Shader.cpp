#include "Shader.h"
#include <iostream>

std::stack<cShader::tShaderEntry> cShader::gShaderStack = std::stack<tShaderEntry>();

const std::string gUniformNames[cShader::eUniformMax] =
{
	"uModelViewMatrix",
	"uProjectionMatrix",
	"uColor"
};

cShader::tShaderEntry::tShaderEntry()
{
	mShader = nullptr;
}

const cShader* cShader::GetBoundShader()
{
	const cShader* shader = nullptr;
	if (!gShaderStack.empty())
	{
		const tShaderEntry& prev_entry = gShaderStack.top();
		shader = prev_entry.mShader;
	}
	return shader;
}

cShader::cShader(void)
{
	mProg = gInvalidIdx;
}

cShader::~cShader(void)
{
	Clear();
}

void cShader::Clear()
{
	if (mProg != gInvalidIdx)
	{
		glDeleteProgram(mProg);
		mProg = gInvalidIdx;
	}
}

bool cShader::BuildShader(const std::string& vs_filename, const std::string& ps_filename)
{
	Clear();

	GLuint vs = LoadShader(vs_filename, GL_VERTEX_SHADER);
	GLuint ps = LoadShader(ps_filename, GL_FRAGMENT_SHADER);
	mProg = CreateShader(vs, ps);

	bool succ = mProg != gInvalidIdx;
	if (succ)
	{
		RecordUniformHandles();
	}

	return succ;
}

GLuint cShader::LoadShader(const std::string& filename, GLenum shader_type)
{
	GLuint shader_handle = glCreateShader(shader_type);

	//read shader file
	std::ifstream shader_file(filename.c_str());
	std::cout << "Compiling shader: " << filename.c_str() << std::endl;

	std::string shader_code((std::istreambuf_iterator<char>(shader_file)),
		(std::istreambuf_iterator<char>()));

	const char* shader_code_str = shader_code.c_str();
	glShaderSource(shader_handle, 1, &shader_code_str, NULL);
	glCompileShader(shader_handle);
	// Check the compile status

	GLint compiled;
	glGetShaderiv(shader_handle, GL_COMPILE_STATUS, &compiled);
	if (!compiled)
	{
		std::cout << "shader compilation failed: " << filename.c_str() << std::endl;

		// get log string length
		GLint max_len = 0;
		glGetShaderiv(shader_handle, GL_INFO_LOG_LENGTH, &max_len);

		char* error_log = new char[max_len];
		glGetShaderInfoLog(shader_handle, max_len, &max_len, error_log);

		std::cout << error_log << std::endl;

		delete[] error_log;
		glDeleteShader(shader_handle);
		shader_handle = gInvalidIdx;
	}

	return shader_handle;
}

GLuint cShader::CreateShader(GLuint& vsh_handle, GLuint& psh_handle)
{
	GLuint shader_program;
	GLint valid_status;

	shader_program = glCreateProgram();
	glAttachShader(shader_program, vsh_handle);
	glAttachShader(shader_program, psh_handle);
	glLinkProgram(shader_program);
	glGetProgramiv(shader_program, GL_LINK_STATUS, &valid_status);

	// valid program
	glGetProgramiv(shader_program, GL_LINK_STATUS, &valid_status);
	if (!valid_status)
	{
		std::cout << "shader linking failed\n";
	}
	else
	{
		glValidateProgram(shader_program);
		glGetProgramiv(shader_program, GL_VALIDATE_STATUS, &valid_status);
		if (!valid_status)
		{
			std::cout << "shader validation failed\n";
		}
	}

	if (!valid_status)
	{
		shader_program = gInvalidIdx;
	}

	return shader_program;
}

void cShader::RecordUniformHandles()
{
	for (int i = 0; i < eUniformMax; ++i)
	{
		const std::string& name = gUniformNames[i];
		GLuint handle = gInvalidIdx;
		GetUniformHandle(handle, name, false);
		mUniformHandles[i] = handle;
	}
}

void cShader::GetAttributeHandle(GLuint& attribute_handle, const std::string& attribute_name)
{
	attribute_handle = glGetAttribLocation(mProg, attribute_name.c_str());
	if (attribute_handle == gInvalidIdx)
	{
		std::cout << "can't find shader attribute " << attribute_name.c_str() << std::endl;
	}
}

void cShader::GetUniformHandle(GLuint& uniform_handle, const std::string& uniform_name, bool check_valid /*= false*/)
{
	uniform_handle = glGetUniformLocation(mProg, uniform_name.c_str());
	if (check_valid && uniform_handle == gInvalidIdx)
	{
		std::cout << "can't find shader uniform parameter " << uniform_name.c_str() << std::endl;
	}
}

void cShader::Bind() const
{
	PushShaderStack();
}

void cShader::Unbind() const
{
	PopShaderStack();
}

void cShader::SetUniform2(GLuint handle, const tVector& data) const
{
	float f_data[] = { static_cast<float>(data[0]),
						static_cast<float>(data[1]) };
	glProgramUniform2fv(mProg, handle, 1, f_data);
}

void cShader::SetUniform3(GLuint handle, const tVector& data) const
{
	float f_data[] = { static_cast<float>(data[0]),
					  static_cast<float>(data[1]),
					  static_cast<float>(data[2]) };
	glProgramUniform3fv(mProg, handle, 1, f_data);
}

void cShader::SetUniform4(GLuint handle, const tVector& data) const
{
	float f_data[] = { static_cast<float>(data[0]),
					  static_cast<float>(data[1]),
					  static_cast<float>(data[2]),
					  static_cast<float>(data[3]) };
	glProgramUniform4fv(mProg, handle, 1, f_data);
}

void cShader::SetUniformMat(GLuint handle, const tMatrix& data) const
{
	float f_data[] = {
		static_cast<float>(data(0, 0)), static_cast<float>(data(1, 0)), static_cast<float>(data(2, 0)), static_cast<float>(data(3, 0)),
		static_cast<float>(data(0, 1)), static_cast<float>(data(1, 1)), static_cast<float>(data(2, 1)), static_cast<float>(data(3, 1)),
		static_cast<float>(data(0, 2)), static_cast<float>(data(1, 2)), static_cast<float>(data(2, 2)), static_cast<float>(data(3, 2)),
		static_cast<float>(data(0, 3)), static_cast<float>(data(1, 3)), static_cast<float>(data(2, 3)), static_cast<float>(data(3, 3)) };
	glProgramUniformMatrix4fv(mProg, handle, 1, false, f_data);
}

bool cShader::HasUniform(eUniform unif) const
{
	return mUniformHandles[unif] != gInvalidIdx;
}

void cShader::SetUniform4(eUniform unif, const tVector& data) const
{
	GLuint handle = mUniformHandles[unif];
	SetUniform4(handle, data);
}

void cShader::SetUniformMat(eUniform unif, const tMatrix& data) const
{
	GLuint handle = mUniformHandles[unif];
	SetUniformMat(handle, data);
}

bool cShader::CheckBoundShader() const
{
	GLint prog;
	glGetIntegerv(GL_CURRENT_PROGRAM, &prog);
	return prog == mProg;
}

GLuint cShader::GetProg() const
{
	return mProg;
}

void cShader::PushShaderStack() const
{
	glUseProgram(mProg);

	tShaderEntry entry;
	entry.mShader = this;
	gShaderStack.push(entry);
}

void cShader::PopShaderStack() const
{
	bool bound = CheckBoundShader();
	if (bound)
	{
		gShaderStack.pop();
		if (!gShaderStack.empty())
		{
			const tShaderEntry& prev_entry = gShaderStack.top();
			const cShader* prev_shader = prev_entry.mShader;
			GLuint prev_prog = prev_shader->GetProg();

			glUseProgram(prev_prog);
		}
		else
		{
			glUseProgram(0);
		}
	}
	else
	{
		throw std::runtime_error("Trying to pop unbound shader from stack.\n");
	}
}