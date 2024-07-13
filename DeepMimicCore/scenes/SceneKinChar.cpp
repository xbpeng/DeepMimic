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
	if (succ)
    {
        succ = BuildControllers();
    }
}

void cSceneKinChar::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
    cScene::ParseArgs(parser);
    ParseCharParams(parser, mCharParams);
    ParseCharCtrlParams(parser, mCtrlParams);
}

void cSceneKinChar::Reset()
{
	ResetCharacters();
}

void cSceneKinChar::Clear()
{
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
	std::vector<std::string> char_files;
	bool succ = parser->ParseStrings("character_files", char_files);

	if (succ)
	{
		std::string state_file = "";
		parser->ParseString("state_file", state_file);

		double init_pos_x = 0;
		parser->ParseDouble("char_init_pos_xs", init_pos_x);

		int num_files = static_cast<int>(char_files.size());
		out_params.resize(num_files);

		for (int i = 0; i < num_files; ++i)
		{
			cKinCharacter::tParams& out_param = out_params[i];
			out_param.mCharFile = char_files[i];
			out_param.mStateFile = state_file;
			out_param.mOrigin[0] = init_pos_x;
		}
	}
	else
	{
		printf("No character files provided\n");
	}
}

void cSceneKinChar::ParseCharCtrlParams(const std::shared_ptr<cArgParser>& parser, std::vector<cKinCtrlBuilder::tCtrlParams>& out_params) const
{
	std::vector<std::string> motion_files;
	parser->ParseStrings("motion_files", motion_files);

	std::string kin_ctrl_str;
	parser->ParseString("kin_ctrl", kin_ctrl_str);

	int num_files = static_cast<int>(motion_files.size());
	out_params.resize(num_files);

	for (int i = 0; i < num_files; ++i)
	{
		auto& ctrl_params = out_params[i];
		const std::string& type_str = kin_ctrl_str;
		cKinCtrlBuilder::ParseCharCtrl(type_str, ctrl_params.mCharCtrl);
		ctrl_params.mCtrlFile = motion_files[i];
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

bool cSceneKinChar::BuildControllers()
{
	int num_char = static_cast<int>(mChars.size());
	bool succ = false;
	for (int i = 0; i < num_char; ++i)
	{
		auto& kin_char = mChars[i];
		mCtrlParams[i].mChar = kin_char;

		std::shared_ptr<cKinController> ctrl;
		succ = cKinCtrlBuilder::BuildController(mCtrlParams[i], ctrl);
		if (succ && ctrl != nullptr)
		{
			kin_char->SetController(ctrl);
		}
		else
		{
			return false;  // Return false if any controller fails to build
		}
	}
	return true;
}