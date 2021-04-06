#pragma once

#include <vector>
#include <string>
#include <memory>

#include "util/MathUtil.h"
#include "anim/KinController.h"

class cKinCtrlBuilder
{
public:

	enum eCharCtrl
	{
		eCharCtrlNone,
		eCharCtrlMotion,
		eCharCtrlClips,
		eCharCtrlMax
	};

	struct tCtrlParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		eCharCtrl mCharCtrl;
		std::string mCtrlFile;

		std::shared_ptr<cKinCharacter> mChar;

		tCtrlParams();
	};
	
	static void ParseCharCtrl(const std::string& char_ctrl_str, eCharCtrl& out_char_ctrl);
	static bool BuildController(const tCtrlParams& params, std::shared_ptr<cKinController>& out_ctrl);
	
protected:

	static bool BuildMotionController(const tCtrlParams& params, std::shared_ptr<cKinController>& out_ctrl);
	static bool BuildClipsController(const tCtrlParams& params, std::shared_ptr<cKinController>& out_ctrl);
};
