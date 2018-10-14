#include "ExpPDController.h"
#include <iostream>

#include "sim/SimCharacter.h"

cExpPDController::cExpPDController()
{
}

cExpPDController::~cExpPDController()
{
}

void cExpPDController::Init(cSimCharacter* character, const Eigen::MatrixXd& pd_params)
{
	cController::Init(character);
	int num_joints = mChar->GetNumJoints();
	mPDCtrls.resize(num_joints);
	assert(pd_params.rows() == num_joints);

	int root_id = mChar->GetRootID();
	for (int j = 0; j < num_joints; ++j)
	{
		cSimBodyJoint& joint = mChar->GetJoint(j);
		if (joint.IsValid() && j != root_id)
		{
			const cPDController::tParams& curr_params = pd_params.row(j);
			cPDController& ctrl = mPDCtrls[j];
			ctrl.Init(mChar, curr_params);
		}
	}

	mValid = true;
}

void cExpPDController::Reset()
{
	cController::Reset();
	for (size_t i = 0; i < mPDCtrls.size(); ++i)
	{
		mPDCtrls[i].Reset();
	}
}

void cExpPDController::Clear()
{
	cController::Clear();
	mPDCtrls.clear();
}

void cExpPDController::Update(double time_step)
{
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(GetNumDof());
	UpdateControlForce(time_step, tau);
	ApplyControlForces(tau);
}

void cExpPDController::UpdateControlForce(double time_step, Eigen::VectorXd& out_tau)
{
	cController::Update(time_step);
	if (time_step > 0)
	{
		CalcControlForces(time_step, out_tau);
	}
}

int cExpPDController::GetNumJoints() const
{
	return mChar->GetNumJoints();
}

int cExpPDController::GetNumDof() const
{
	return mChar->GetNumDof();
}

void cExpPDController::GetTargetTheta(int joint_id, Eigen::VectorXd& out_theta) const
{
	const cPDController& pd_ctrl = GetPDCtrl(joint_id);
	pd_ctrl.GetTargetTheta(out_theta);
}

void cExpPDController::SetTargetTheta(int joint_id, const Eigen::VectorXd& theta)
{
	cPDController& pd_ctrl = GetPDCtrl(joint_id);
	pd_ctrl.SetTargetTheta(theta);
}

void cExpPDController::GetTargetVel(int joint_id, Eigen::VectorXd& out_vel) const
{
	const cPDController& pd_ctrl = GetPDCtrl(joint_id);
	pd_ctrl.GetTargetVel(out_vel);
}

void cExpPDController::SetTargetVel(int joint_id, const Eigen::VectorXd& vel)
{
	cPDController& pd_ctrl = GetPDCtrl(joint_id);
	pd_ctrl.SetTargetVel(vel);
}

bool cExpPDController::UseWorldCoord(int joint_id) const
{
	const cPDController& pd_ctrl = GetPDCtrl(joint_id);
	return pd_ctrl.UseWorldCoord();
}

void cExpPDController::SetUseWorldCoord(int joint_id, bool use)
{
	cPDController& pd_ctrl = GetPDCtrl(joint_id);
	pd_ctrl.SetUseWorldCoord(use);
}

void cExpPDController::SetKp(int joint_id, double kp)
{
	cPDController& pd_ctrl = GetPDCtrl(joint_id);
	pd_ctrl.SetKp(kp);
}

void cExpPDController::SetKd(int joint_id, double kd)
{
	cPDController& pd_ctrl = GetPDCtrl(joint_id);
	pd_ctrl.SetKd(kd);
}

bool cExpPDController::IsValidPDCtrl(int joint_id) const
{
	const cPDController& pd_ctrl = GetPDCtrl(joint_id);
	return pd_ctrl.IsValid();
}

cPDController& cExpPDController::GetPDCtrl(int joint_id)
{
	assert(joint_id >= 0 && joint_id < GetNumJoints());
	return mPDCtrls[joint_id];
}

const cPDController& cExpPDController::GetPDCtrl(int joint_id) const
{
	assert(joint_id >= 0 && joint_id < GetNumJoints());
	return mPDCtrls[joint_id];
}

void cExpPDController::CalcControlForces(double time_step, Eigen::VectorXd& out_tau)
{
	for (int i = 0; i < static_cast<int>(mPDCtrls.size()); ++i)
	{
		cPDController& pd_ctrl = GetPDCtrl(i);
		if (pd_ctrl.IsValid())
		{
			pd_ctrl.UpdateControlForce(time_step, out_tau);
		}
	}
}

void cExpPDController::ApplyControlForces(const Eigen::VectorXd& tau)
{
	mChar->ApplyControlForces(tau);
}