#include "SceneKinChar.h"

cSceneKinChar::cSceneKinChar()
{
}

cSceneKinChar::~cSceneKinChar()
{
}

void cSceneKinChar::Init()
{
	bool succ = BuildCharacters();
}

void cSceneKinChar::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScene::ParseArgs(parser);
	ParseCharParams(parser, mCharParams);
}

void cSceneKinChar::Reset()
{
	ResetCharacters();
}

void cSceneKinChar::Clear()
{
	// int num_char = static_cast<int>(mCharParams.size());
	// for (int i=0; i < num_char; ++i)
	// {
	// 	mChars[i].reset();
	// }
	mChars.clear();
}

void cSceneKinChar::Update(double time_elapsed)
{
	UpdateCharacters(time_elapsed);
}

int cSceneKinChar::GetNumCharacter()
{
	return static_cast<int>(mCharParams.size());
}
const std::shared_ptr<cKinCharacter>& cSceneKinChar::GetCharacter(int id) const
{ 
	return mChars[id];
}

tVector cSceneKinChar::GetCharPos() const
{
	return GetCharacter(0)->GetRootPos();
}

double cSceneKinChar::GetTime() const
{
	return GetCharacter(0)->GetTime();
}

std::string cSceneKinChar::GetName() const
{
	return "Kinematic Char";
}

void cSceneKinChar::ParseCharParams(const std::shared_ptr<cArgParser>& parser, std::vector<cKinCharacter::tParams>& out_params) const
{
	bool succ = true;

	std::vector<std::string> char_files;
	succ = parser->ParseStrings("character_files", char_files);

	std::string state_file = "";
	succ = parser->ParseString("state_file", state_file);

	std::vector<std::string> motion_files;
	succ = parser->ParseStrings("motion_files", motion_files);

	double init_pos_xs;
	parser->ParseDouble("char_init_pos_x", init_pos_xs);

	int num_files = static_cast<int>(char_files.size());
	out_params.resize(num_files);

	for (int i = 0; i < num_files; ++i)
	{
		cKinCharacter::tParams& params = out_params[i];
		params.mCharFile = char_files[i];
		params.mMotionFile = motion_files[i];
		params.mStateFile = state_file;
		params.mOrigin[0] = init_pos_xs;
	}
}

bool cSceneKinChar::BuildCharacters()
{
	int num_char = static_cast<int>(mCharParams.size());
	bool succ = false;
	mChars.clear();
	for (int i=0; i < num_char; ++i)
	{
		std::shared_ptr<cKinCharacter> curr_char;
		auto& params = mCharParams[i];
		params.mID = i;
		params.mLoadDrawShapes = true;
		succ = BuildCharacter(params, curr_char);
		mChars.push_back(curr_char);
	}	
	return succ;
}

bool cSceneKinChar::BuildCharacter(const cKinCharacter::tParams& params, std::shared_ptr<cKinCharacter>& out_char) const
{
	out_char = std::shared_ptr<cKinCharacter>(new cKinCharacter());
	bool succ = out_char->Init(params);
	return succ;
}

void cSceneKinChar::ResetCharacters()
{
	int num_char = static_cast<int>(mCharParams.size());
	for (int i=0; i < num_char; ++i)
	{
		mChars[i]->Reset();
	}
}

void cSceneKinChar::UpdateCharacters(double timestep)
{
	int num_char = static_cast<int>(mCharParams.size());
	for (int i=0; i < num_char; ++i)
	{
		mChars[i]->Update(timestep);
	}
}

