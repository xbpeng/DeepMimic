#include "anim/KinCtrlBuilder.h"
#include "anim/MotionController.h"
#include "anim/ClipsController.h"

const std::string gCharCtrlName[cKinCtrlBuilder::eCharCtrlMax] =
{
	"none",
	"motion",
	"clips"
};

cKinCtrlBuilder::tCtrlParams::tCtrlParams()
{
	mCharCtrl = eCharCtrlNone;
	mCtrlFile = "";
	mChar = nullptr;
}

void cKinCtrlBuilder::ParseCharCtrl(const std::string& char_ctrl_str, eCharCtrl& out_char_ctrl)
{
	bool found = false;
	if (char_ctrl_str == "" || char_ctrl_str == "none")
	{
		out_char_ctrl = eCharCtrlNone;
		found = true;
	}
	else
	{
		for (int i = 0; i < eCharCtrlMax; ++i)
		{
			const std::string& name = gCharCtrlName[i];
			if (char_ctrl_str == name)
			{
				out_char_ctrl = static_cast<eCharCtrl>(i);
				found = true;
				break;
			}
		}
	}

	if (!found)
	{
		assert(false && "Unsupported kinematic controller\n"); // unsupported character controller
	}
}

bool cKinCtrlBuilder::BuildController(const tCtrlParams& params, std::shared_ptr<cKinController>& out_ctrl)
{
	bool succ = true;

	switch (params.mCharCtrl)
	{
	case eCharCtrlNone:
		break;
	case eCharCtrlMotion:
		succ = BuildMotionController(params, out_ctrl);
		break;
	case eCharCtrlClips:
		succ = BuildClipsController(params, out_ctrl);
		break;
	default:
		assert(false && "Failed Building Unsupported Controller\n"); // unsupported controller
		break;
	}

	return succ;
}

bool cKinCtrlBuilder::BuildMotionController(const tCtrlParams& params, std::shared_ptr<cKinController>& out_ctrl)
{
	bool succ = true;
	std::shared_ptr<cMotionController> ctrl = std::shared_ptr<cMotionController>(new cMotionController());
	ctrl->Init(params.mChar.get(), params.mCtrlFile);

	out_ctrl = ctrl;
	return succ;
}

bool cKinCtrlBuilder::BuildClipsController(const tCtrlParams& params, std::shared_ptr<cKinController>& out_ctrl)
{
	bool succ = true;
	std::shared_ptr<cClipsController> ctrl = std::shared_ptr<cClipsController>(new cClipsController());
	ctrl->Init(params.mChar.get(), params.mCtrlFile);

	out_ctrl = ctrl;
	return succ;
}