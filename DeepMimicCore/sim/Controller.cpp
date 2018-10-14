#include "Controller.h"
#include "sim/SimCharacter.h"
#include "util/FileUtil.h"

cController::cController()
{
	mChar = nullptr;
}

cController::~cController()
{

}


void cController::Init(cSimCharacter* character)
{
	assert(character != nullptr);
	Clear();
	mChar = character;
}

void cController::Reset()
{
	SetMode(eModeActive);
}


void cController::Clear()
{
	mValid = false;
	mChar = nullptr;
	SetMode(eModeActive);
}

void cController::Update(double time_step)
{
	// *whistle whistle*....nothing to see here
}

bool cController::IsValid() const
{
	return mValid;
}

bool cController::LoadParams(const std::string& param_file)
{
	std::ifstream f_stream(param_file);
	Json::Reader reader;
	Json::Value root;
	bool succ = reader.parse(f_stream, root);
	f_stream.close();

	if (succ)
	{
		succ &= ParseParams(root);
	}
	
	if (!succ)
	{
		printf("Failed to load params from %s\n", param_file.c_str());
		assert(false);
	}

	return succ;
}

int cController::GetNumOptParams() const
{
	return 0;
}

void cController::BuildOptParams(Eigen::VectorXd& out_params) const
{
}

void cController::SetOptParams(const Eigen::VectorXd& params)
{
}

void cController::SetOptParams(const Eigen::VectorXd& params, Eigen::VectorXd& out_params) const
{
	out_params = params;
}

void cController::FetchOptParamScale(Eigen::VectorXd& out_scale) const
{
}

void cController::OutputOptParams(const std::string& file, const Eigen::VectorXd& params) const
{
	FILE* f = cFileUtil::OpenFile(file, "w");
	if (f != nullptr)
	{
		OutputOptParams(f, params);
		cFileUtil::CloseFile(f);
	}
}

void cController::OutputOptParams(FILE* f, const Eigen::VectorXd& params) const
{
}

void cController::SetActive(bool active)
{
	SetMode((active) ? eModeActive : eModeInactive);
}

bool cController::IsActive() const
{
	return mMode != eModeInactive;
}

void cController::SetMode(eMode mode)
{
	mMode = mode;
}

const cSimCharacter* cController::GetChar() const
{
	return mChar;
}

bool cController::ParseParams(const Json::Value& json)
{
	return true;
}