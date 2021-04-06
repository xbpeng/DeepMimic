#include "SceneKinChar.h"

cSceneKinChar::cSceneKinChar()
{
}

cSceneKinChar::~cSceneKinChar()
{
}

void cSceneKinChar::Init()
{
	BuildCharacter();
	BuildController();
}

void cSceneKinChar::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScene::ParseArgs(parser);
	ParseCharParams(parser, mCharParams);
	ParseCharCtrlParams(parser, mCtrlParams);
}

void cSceneKinChar::Reset()
{
	ResetCharacter();
}

void cSceneKinChar::Clear()
{
	mChar.reset();
}

void cSceneKinChar::Update(double time_elapsed)
{
	UpdateCharacter(time_elapsed);
}

const std::shared_ptr<cKinCharacter>& cSceneKinChar::GetCharacter() const
{
	return mChar;
}

tVector cSceneKinChar::GetCharPos() const
{
	return GetCharacter()->GetRootPos();
}

double cSceneKinChar::GetTime() const
{
	return GetCharacter()->GetTime();
}

std::string cSceneKinChar::GetName() const
{
	return "Kinematic Char";
}

void cSceneKinChar::ParseCharParams(const std::shared_ptr<cArgParser>& parser, cKinCharacter::tParams& out_param) const
{
	std::string char_file;
	bool succ = parser->ParseString("character_file", char_file);

	if (succ)
	{
		std::string state_file = "";
		parser->ParseString("state_file", state_file);

		double init_pos_x = 0;
		parser->ParseDouble("char_init_pos_xs", init_pos_x);

		out_param.mCharFile = char_file;
		out_param.mStateFile = state_file;
		out_param.mOrigin[0] = init_pos_x;
	}
	else
	{
		printf("No character files provided\n");
	}
}

void cSceneKinChar::ParseCharCtrlParams(const std::shared_ptr<cArgParser>& parser, cKinCtrlBuilder::tCtrlParams& out_params) const
{
	std::string motion_file;
	parser->ParseString("motion_file", motion_file);

	std::string kin_ctrl_str;
	parser->ParseString("kin_ctrl", kin_ctrl_str);

	auto& ctrl_params = out_params;
	const std::string& type_str = kin_ctrl_str;
	cKinCtrlBuilder::ParseCharCtrl(type_str, ctrl_params.mCharCtrl);
	ctrl_params.mCtrlFile = motion_file;
}

bool cSceneKinChar::BuildCharacter()
{
	mChar.reset();

	auto& params = mCharParams;
	params.mID = 0;
	params.mLoadDrawShapes = true;

	mChar = std::shared_ptr<cKinCharacter>(new cKinCharacter());
	bool succ = mChar->Init(params);

	return succ;
}

void cSceneKinChar::ResetCharacter()
{
	mChar->Reset();
}

void cSceneKinChar::UpdateCharacter(double timestep)
{
	mChar->Update(timestep);
}

bool cSceneKinChar::BuildController()
{
	const auto& kin_char = GetCharacter();
	mCtrlParams.mChar = kin_char;

	std::shared_ptr<cKinController> ctrl;
	bool succ = cKinCtrlBuilder::BuildController(mCtrlParams, ctrl);
	if (succ && ctrl != nullptr)
	{
		kin_char->SetController(ctrl);
	}

	return succ;
}