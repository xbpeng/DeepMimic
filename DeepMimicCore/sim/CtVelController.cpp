#include "CtVelController.h"
#include "sim/SimCharacter.h"

cCtVelController::cCtVelController() : cCtPDController()
{
}

cCtVelController::~cCtVelController()
{
}

std::string cCtVelController::GetName() const
{
	return "ct_vel";
}

int cCtVelController::GetCtrlParamSize(int joint_id) const
{
	int param_size = 0;
	int root_id = mChar->GetRootID();

	if (joint_id != root_id)
	{
		const auto& joint_mat = mChar->GetJointMat();
		param_size = cCtCtrlUtil::GetParamDimVel(joint_mat, joint_id);
	}

	return param_size;
}

void cCtVelController::SetupPDControllers(const Json::Value& json, const tVector& gravity)
{
	cCtPDController::SetupPDControllers(json, gravity);

	int num_joints = mPDCtrl.GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		bool valid_pd = mPDCtrl.IsValidPDCtrl(j);
		if (valid_pd)
		{
			mPDCtrl.SetKp(j, 0);
		}
	}
}

void cCtVelController::ApplyAction(const Eigen::VectorXd& action)
{
	cCtController::ApplyAction(action);

	int root_size = mChar->GetParamSize(mChar->GetRootID());
	int num_joints = mChar->GetNumJoints();
	int ctrl_offset = GetActionCtrlOffset();

	for (int j = 0; j < num_joints; ++j)
	{
		if (mPDCtrl.IsValidPDCtrl(j))
		{
			int param_offset = mChar->GetParamOffset(j);
			int param_size = mChar->GetParamSize(j);

			param_offset -= root_size;
			param_offset += ctrl_offset;
			Eigen::VectorXd theta_dot = mAction.segment(param_offset, param_size);
			mPDCtrl.SetTargetVel(j, theta_dot);
		}
	}
}

void cCtVelController::BuildJointActionBounds(int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	cCtCtrlUtil::BuildBoundsVel(joint_mat, joint_id, out_min, out_max);
}

void cCtVelController::BuildJointActionOffsetScale(int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	cCtCtrlUtil::BuildOffsetScaleVel(joint_mat, joint_id, out_offset, out_scale);
}