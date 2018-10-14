#include "PDController.h"
#include <iostream>

#include "sim/SimCharacter.h"
#include "util/FileUtil.h"

const std::string gPDParamKeys[cPDController::eParamMax] =
{
	"JointID",
	"Kp",
	"Kd",
	"TargetTheta0",
	"TargetTheta1",
	"TargetTheta2",
	"TargetTheta3",
	"TargetTheta4",
	"TargetTheta5",
	"TargetTheta6",
	"TargetVel0",
	"TargetVel1",
	"TargetVel2",
	"TargetVel3",
	"TargetVel4",
	"TargetVel5",
	"TargetVel6",
	"UseWorldCoord"
};

bool cPDController::LoadParams(const std::string& file, Eigen::MatrixXd& out_buffer)
{
	std::ifstream f_stream(file);
	Json::Reader reader;
	Json::Value root;
	bool succ = reader.parse(f_stream, root);
	f_stream.close();

	if (succ)
	{
		succ &= LoadParams(root, out_buffer);
	}
	
	if (!succ)
	{
		printf("Failed to load PD controller parameters from %s\n", file.c_str());
	}

	return succ;
}

bool cPDController::LoadParams(const Json::Value& root, Eigen::MatrixXd& out_buffer)
{
	bool succ = true;
	int num_ctrls = root.size();
	out_buffer.resize(num_ctrls, eParamMax);

	for (int i = 0; i < num_ctrls; ++i)
	{
		tParams curr_params;
		const Json::Value& json_pd_ctrl = root.get(i, 0);
		bool succ_def = ParsePDParams(json_pd_ctrl, curr_params);
		if (succ_def)
		{
			int joint_id = i;
			curr_params[eParamJointID] = i;
			out_buffer.row(i) = curr_params;
		}
		else
		{
			succ = false;
			break;
		}
	}
	return succ;
}

bool cPDController::ParsePDParams(const Json::Value& root, tParams& out_params)
{
	bool succ = true;

	out_params.setZero();
	for (int i = 0; i < eParamMax; ++i)
	{
		const std::string& curr_key = gPDParamKeys[i];
		if (!root[curr_key].isNull() && root[curr_key].isNumeric())
		{
			Json::Value json_val = root[curr_key];
			double val = json_val.asDouble();
			out_params[i] = val;
		}
	}

	return succ;
}

cPDController::cPDController()
	: cController()
{
	Clear();
}

cPDController::~cPDController()
{
}

void cPDController::Init(cSimCharacter* character, const tParams& params)
{
	cController::Init(character);
	mParams = params;

	int joint_dim = GetJointDim();
	Eigen::VectorXd target_pose0;
	GetTargetTheta(target_pose0);
	SetTargetTheta(target_pose0);

	mValid = true;
}

void cPDController::Clear()
{
	cController::Clear();

	mParams.setZero();
	mParams[eParamJointID] = static_cast<double>(cKinTree::gInvalidJointID);
}

void cPDController::Update(double time_step)
{
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(mChar->GetNumDof());
	UpdateControlForce(time_step, tau);
	if (IsActive())
	{
		ApplyControlForces(tau);
	}
}

void cPDController::UpdateControlForce(double time_step, Eigen::VectorXd& out_tau)
{
	cController::Update(time_step);
	if (IsActive())
	{
		Eigen::VectorXd joint_tau;
		CalcJointTau(time_step, joint_tau);

		int joint_id = GetJointID();
		int param_offset = mChar->GetParamOffset(joint_id);
		int param_size = mChar->GetParamSize(joint_id);
		assert(param_size == joint_tau.size());

		out_tau.segment(param_offset, param_size) += joint_tau;
	}
}

cSimBodyJoint& cPDController::GetJoint()
{
	return mChar->GetJoint(GetJointID());
}

const cSimBodyJoint& cPDController::GetJoint() const
{
	return mChar->GetJoint(GetJointID());
}

void cPDController::SetKp(double kp)
{
	mParams[eParamKp] = kp;
}

double cPDController::GetKp() const
{
	return mParams[eParamKp];
}

double cPDController::GetTorqueLimit() const
{
	return GetJoint().GetTorqueLimit();
}

double cPDController::GetForceLimit() const
{
	return GetJoint().GetForceLimit();
}

void cPDController::SetKd(double kd)
{
	mParams[eParamKd] = kd;
}

double cPDController::GetKd() const
{
	return mParams[eParamKd];
}

void cPDController::SetTargetTheta(const Eigen::VectorXd& theta)
{
	int theta_size = static_cast<int>(theta.size());
	int joint_dim = GetJointDim();
	assert(theta_size == joint_dim);
	assert(theta_size < 7);
	Eigen::VectorXd theta_proc = theta;
	PostProcessTargetPose(theta_proc);
	mParams.segment(eParamTargetTheta0, theta_size) = theta_proc;
}

void cPDController::SetTargetVel(const Eigen::VectorXd& vel)
{
	int vel_size = static_cast<int>(vel.size());
	int joint_dim = GetJointDim();
	assert(vel_size == joint_dim);
	assert(vel_size < 7);
	Eigen::VectorXd vel_proc = vel;
	PostProcessTargetVel(vel_proc);
	mParams.segment(eParamTargetVel0, vel_size) = vel_proc;
}

void cPDController::SetUseWorldCoord(bool use)
{
	mParams[eParamUseWorldCoord] = (use) ? 1 : 0;
}

bool cPDController::UseWorldCoord() const
{
	return mParams[eParamUseWorldCoord] != 0;
}

void cPDController::GetTargetTheta(Eigen::VectorXd& out_theta) const
{
	const cSimBodyJoint& joint = GetJoint();
	cKinTree::eJointType joint_type = joint.GetType();
	out_theta = mParams.segment(eParamTargetTheta0, GetJointDim());

	if (UseWorldCoord() && joint_type == cKinTree::eJointTypeRevolute)
	{
		tVector axis_world;
		double theta_world;
		joint.CalcWorldRotation(axis_world, theta_world);

		Eigen::VectorXd pose;
		joint.BuildPose(pose);
		double theta_rel = pose[0];
		tVector axis_rel = joint.GetAxisRel();

		tVector axis_ref = joint.CalcAxisWorld();
		double theta_offset = axis_world.dot(axis_ref) * theta_world
							- axis_rel.dot(axis_ref) * theta_rel;
		out_theta(0) -= theta_offset;
	}

	if (UseWorldCoord() && joint_type == cKinTree::eJointTypeSpherical)
	{
		Eigen::VectorXd pose;
		joint.BuildPose(pose);

		tQuaternion world_rot = joint.CalcWorldRotation();
		tQuaternion rel_rot = cMathUtil::VecToQuat(pose);

		tQuaternion tar_q = cMathUtil::VecToQuat(out_theta);
		tQuaternion diff = world_rot.conjugate() * tar_q; // diference between target world and world
		tar_q = rel_rot * diff; // Add this difference to current relative rotation
		out_theta = cMathUtil::QuatToVec(tar_q);
	}

	if (UseWorldCoord() && ((joint_type != cKinTree::eJointTypeRevolute) && (joint_type != cKinTree::eJointTypeSpherical)))
	{
		printf("Only revolute joints support world space targets\n");
		assert(false);
	}
}

void cPDController::GetTargetVel(Eigen::VectorXd& out_vel) const
{
	out_vel = mParams.segment(eParamTargetVel0, GetJointDim());
}


bool cPDController::IsActive() const
{
	bool active = cController::IsActive();
	return active;
}

int cPDController::GetJointDim() const
{
	return mChar->GetParamSize(GetJointID());
}

int cPDController::GetJointID() const
{
	return static_cast<int>(mParams[eParamJointID]);
}

void cPDController::ApplyControlForces(const Eigen::VectorXd& tau)
{
	mChar->ApplyControlForces(tau);
}

void cPDController::CalcJointTau(double time_step, Eigen::VectorXd& out_joint_tau)
{
	const cSimBodyJoint& joint = GetJoint();
	cKinTree::eJointType joint_type = joint.GetType();
	switch (joint_type)
	{
	case cKinTree::eJointTypeRevolute:
		CalcJointTauRevolute(time_step, out_joint_tau);
		break;
	case cKinTree::eJointTypePlanar:
		CalcJointTauPlanar(time_step, out_joint_tau);
		break;
	case cKinTree::eJointTypePrismatic:
		CalcJointTauPrismatic(time_step, out_joint_tau);
		break;
	case cKinTree::eJointTypeFixed:
		CalcJointTauFixed(time_step, out_joint_tau);
		break;
	case cKinTree::eJointTypeSpherical:
		CalcJointTauSpherical(time_step, out_joint_tau);
		break;
	default:
		assert(false); // unsupported joint type
		break;
	}
}

void cPDController::CalcJointTauRevolute(double time_step, Eigen::VectorXd& out_joint_tau)
{
	const cSimBodyJoint& joint = GetJoint();

	double kp = GetKp();
	double kd = GetKd();

	Eigen::VectorXd tar_pose;
	Eigen::VectorXd tar_vel;
	GetTargetTheta(tar_pose);
	GetTargetVel(tar_vel);

	Eigen::VectorXd pose;
	joint.BuildPose(pose);
	double theta = pose[0];

	Eigen::VectorXd joint_vel;
	joint.BuildVel(joint_vel);
	double vel = joint_vel[0];

	double theta_err = tar_pose[0] - theta;
	double vel_err = tar_vel[0] - vel;
	double t = kp * theta_err + kd * vel_err;
	//std::cout << "Revolute: theta: " << theta << " theta err: " << theta_err << " vel_err: " << vel_err << std::endl;

	int dim = GetJointDim();
	out_joint_tau = Eigen::VectorXd::Zero(dim);
	out_joint_tau(0) = t;
}

void cPDController::CalcJointTauPlanar(double time_step, Eigen::VectorXd& out_joint_tau)
{
	assert(false); // unsupported
	int dim = GetJointDim();
	out_joint_tau = Eigen::VectorXd::Zero(dim);
}

void cPDController::CalcJointTauPrismatic(double time_step, Eigen::VectorXd& out_joint_tau)
{
	const cSimBodyJoint& joint = GetJoint();

	double kp = GetKp();
	double kd = GetKd();

	Eigen::VectorXd tar_pose;
	Eigen::VectorXd tar_vel;
	GetTargetTheta(tar_pose);
	GetTargetVel(tar_vel);

	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	joint.BuildPose(pose);
	joint.BuildVel(vel);

	double theta_err = tar_pose[0] - pose[0];
	double vel_err = tar_vel[0] - vel[0];
	double t = kp * theta_err + kd * vel_err;

	int dim = GetJointDim();
	out_joint_tau = Eigen::VectorXd::Zero(dim);
	out_joint_tau(0) = t;
}

void cPDController::CalcJointTauFixed(double time_step, Eigen::VectorXd& out_joint_tau)
{
	int dim = GetJointDim();
	out_joint_tau = Eigen::VectorXd::Zero(dim);
}

void cPDController::CalcJointTauSpherical(double time_step, Eigen::VectorXd& out_joint_tau)
{
	int dim = GetJointDim();
	out_joint_tau = Eigen::VectorXd::Zero(dim);
	const cSimBodyJoint& joint = GetJoint();

	Eigen::VectorXd tar_theta;
	Eigen::VectorXd tar_vel;
	GetTargetTheta(tar_theta);
	GetTargetVel(tar_vel);

	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	joint.BuildPose(pose);
	joint.BuildVel(vel);

	tQuaternion tar_q = cMathUtil::VecToQuat(tar_theta);
	tQuaternion q = cMathUtil::VecToQuat(pose);
	tQuaternion q_err = q.conjugate() * tar_q;

	double kp = GetKp();
	double kd = GetKd();
	tVector torque = tVector::Zero();

	tVector axis;
	double theta;
	cMathUtil::QuaternionToAxisAngle(q_err, axis, theta);
	torque = (kp * theta) * axis;

	torque.segment(0, 3) += kd * (tar_vel.segment(0, 3) - vel.segment(0, 3));
	out_joint_tau = torque;
}

void cPDController::PostProcessTargetPose(Eigen::VectorXd& out_pose) const
{
	const cSimBodyJoint& joint = GetJoint();
	int joint_dim = GetJointDim();
	assert(out_pose.size() == joint_dim);

	cKinTree::eJointType joint_type = joint.GetType();
	if (joint_type == cKinTree::eJointTypeSpherical)
	{
		if (out_pose.squaredNorm() == 0)
		{
			out_pose(0) = 1;
		}
		else
		{
			out_pose.normalize();
		}
	}
}

void cPDController::PostProcessTargetVel(Eigen::VectorXd& out_vel) const
{
	const cSimBodyJoint& joint = GetJoint();
	int joint_dim = GetJointDim();
	assert(out_vel.size() == joint_dim);

	cKinTree::eJointType joint_type = joint.GetType();
	if (joint_type == cKinTree::eJointTypeSpherical)
	{
		out_vel(joint_dim - 1) = 0;
	}
}
