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

void cCtPDController::ConvertActionToTargetPose(int joint_id, Eigen::VectorXd& out_theta) const
{
#if defined(ENABLE_PD_SPHERE_AXIS)
	cKinTree::eJointType joint_type = GetJointType(joint_id);
	if (joint_type == cKinTree::eJointTypeSpherical)
	{
		double rot_theta = out_theta[0];
		tVector axis = tVector(out_theta[1], out_theta[2], out_theta[3], 0);
		if (axis.squaredNorm() == 0)
		{
			axis[2] = 1;
		}

		axis.normalize();
		tQuaternion quat = cMathUtil::AxisAngleToQuaternion(axis, rot_theta);

		if (FlipStance())
		{
			cKinTree::eJointType joint_type = GetJointType(joint_id);
			if (joint_type == cKinTree::eJointTypeSpherical)
			{
				quat = cMathUtil::MirrorQuaternion(quat, cMathUtil::eAxisZ);
			}
		}
		out_theta = cMathUtil::QuatToVec(quat);
	}
#endif
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
	int root_size = mChar->GetParamSize(root_id);
	int num_joints = mChar->GetNumJoints();
	int ctrl_offset = GetActionCtrlOffset();

	for (int j = root_id + 1; j < num_joints; ++j)
	{
		if (mPDCtrl.IsValidPDCtrl(j))
		{
			int retarget_joint = RetargetJointID(j);
			int param_offset = mChar->GetParamOffset(retarget_joint);
			int param_size = mChar->GetParamSize(retarget_joint);

			param_offset -= root_size;
			param_offset += ctrl_offset;
			Eigen::VectorXd theta = targets.segment(param_offset, param_size);
			ConvertActionToTargetPose(j, theta);
			mPDCtrl.SetTargetTheta(j, theta);
		}
	}
}