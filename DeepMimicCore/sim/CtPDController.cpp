#include "CtPDController.h"
#include "sim/SimCharacter.h"

const std::string gPDControllersKey = "PDControllers";

cCtPDController::cCtPDController() : cCtController()
{
	mGravity = gGravity;
}

cCtPDController::~cCtPDController()
{
}

void cCtPDController::Reset()
{
	cCtController::Reset();
	mPDCtrl.Reset();
}

void cCtPDController::Clear()
{
	cCtController::Clear();
	mPDCtrl.Clear();
}

void cCtPDController::SetGravity(const tVector& gravity)
{
	mGravity = gravity;
}

std::string cCtPDController::GetName() const
{
	return "ct_pd";
}

void cCtPDController::SetupPDControllers(const Json::Value& json, const tVector& gravity)
{
	Eigen::MatrixXd pd_params;
	bool succ = false;
	if (!json[gPDControllersKey].isNull())
	{
		succ = cPDController::LoadParams(json[gPDControllersKey], pd_params);
	}

	if (succ)
	{
		mPDCtrl.Init(mChar, pd_params, gravity);
	}

	mValid = succ;
	if (!mValid)
	{
		printf("Failed to initialize Ct-PD controller\n");
		mValid = false;
	}
}

bool cCtPDController::ParseParams(const Json::Value& json)
{
	bool succ = cCtController::ParseParams(json);
	SetupPDControllers(json, mGravity);
	return succ;
}

int cCtPDController::GetCtrlParamSize(int joint_id) const
{
	int param_size = 0;
	int root_id = mChar->GetRootID();

	if (joint_id != root_id)
	{
		const auto& joint_mat = mChar->GetJointMat();
		param_size = cCtCtrlUtil::GetParamDimPD(joint_mat, joint_id);
	}

	return param_size;
}

double cCtPDController::GetMaxPDExpVal() const
{
	return cCtCtrlUtil::gMaxPDExpVal;
}

void cCtPDController::UpdateBuildTau(double time_step, Eigen::VectorXd& out_tau)
{
	UpdatePDCtrls(time_step, out_tau);
}

void cCtPDController::UpdatePDCtrls(double time_step, Eigen::VectorXd& out_tau)
{
	int num_dof = mChar->GetNumDof();
	out_tau = Eigen::VectorXd::Zero(num_dof);
	mPDCtrl.UpdateControlForce(time_step, out_tau);
}

void cCtPDController::ApplyAction(const Eigen::VectorXd& action)
{
	cCtController::ApplyAction(action);
	SetPDTargets(action);
}

void cCtPDController::BuildJointActionBounds(int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	cCtCtrlUtil::BuildBoundsPD(joint_mat, joint_id, out_min, out_max);
}

void cCtPDController::BuildJointActionOffsetScale(int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	cCtCtrlUtil::BuildOffsetScalePD(joint_mat, joint_id, out_offset, out_scale);
}

void cCtPDController::ConvertActionToTargetPose(int joint_id, const Eigen::VectorXd& action_params, Eigen::VectorXd& out_theta) const
{
	const double max_len = GetMaxPDExpVal();

	cKinTree::eJointType joint_type = GetJointType(joint_id);
	if (joint_type == cKinTree::eJointTypeSpherical)
	{
		tVector exp_map = tVector(action_params[0], action_params[1], action_params[2], 0);
		double len = exp_map.norm();
		if (len > max_len)
		{
			exp_map *= max_len / len;
		}

		tQuaternion quat = cMathUtil::ExpMapToQuaternion(exp_map);
		out_theta = cMathUtil::QuatToVec(quat);
	}
	else
	{
		out_theta = action_params;
	}
}

cKinTree::eJointType cCtPDController::GetJointType(int joint_id) const
{
	const cPDController& ctrl = mPDCtrl.GetPDCtrl(joint_id);
	const cSimBodyJoint& joint = ctrl.GetJoint();
	cKinTree::eJointType joint_type = joint.GetType();
	return joint_type;
}

void cCtPDController::SetPDTargets(const Eigen::VectorXd& targets)
{
	int root_id = mChar->GetRootID();
	int num_joints = mChar->GetNumJoints();

	for (int j = root_id + 1; j < num_joints; ++j)
	{
		if (mPDCtrl.IsValidPDCtrl(j))
		{
			int retarget_joint = RetargetJointID(j);
			int param_offset = GetCtrlParamOffset(retarget_joint);
			int param_size = GetCtrlParamSize(retarget_joint);
			auto curr_params = targets.segment(param_offset, param_size);

			Eigen::VectorXd theta;
			ConvertActionToTargetPose(j, curr_params, theta);

			mPDCtrl.SetTargetTheta(j, theta);
		}
	}
}