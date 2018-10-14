#pragma once

#include <vector>
#include <string>
#include <memory>

#include "sim/SimCharacter.h"
#include "sim/DeepMimicCharController.h"
#include "sim/Ground.h"

class cCtrlBuilder
{
public:

	enum eCharCtrl
	{
		eCharCtrlNone,
		eCharCtrlCt,
		eCharCtrlCtPD,
		eCharCtrlCtVel,
		eCharCtrlMax
	};

	struct tCtrlParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		eCharCtrl mCharCtrl;
		std::string mCtrlFile;

		std::shared_ptr<cSimCharacter> mChar;
		std::shared_ptr<cGround> mGround;
		tVector mGravity;

		tCtrlParams();
	};
	
	static void ParseCharCtrl(const std::string& char_ctrl_str, eCharCtrl& out_char_ctrl);
	static bool BuildController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	
protected:

	static bool BuildCtController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtPDController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtVelController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtTargetController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);

	static bool BuildDogController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildRaptorController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);

	static bool BuildBiped3DStepController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildBiped3DSymStepController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);

	static bool BuildCtTarPoseController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtSymPDController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);

	static bool BuildCtHeadingController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtHeadingPDController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtSymHeadingPDController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtStrikePDController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtStrikeObjPDController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtCmdPDController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
};
