#pragma once

#include "sim/PDController.h"

//#define IMP_PD_CTRL_PROFILER

class cExpPDController : public cController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cExpPDController();
	virtual ~cExpPDController();

	virtual void Init(cSimCharacter* character, const Eigen::MatrixXd& pd_params);
	virtual void Reset();
	virtual void Clear();
	virtual void Update(double time_step);
	virtual void UpdateControlForce(double time_step, Eigen::VectorXd& out_tau);

	virtual int GetNumJoints() const;
	virtual int GetNumDof() const;

	virtual void GetTargetTheta(int joint_id, Eigen::VectorXd& out_theta) const;
	virtual void SetTargetTheta(int joint_id, const Eigen::VectorXd& theta);
	virtual void GetTargetVel(int joint_id, Eigen::VectorXd& out_vel) const;
	virtual void SetTargetVel(int joint_id, const Eigen::VectorXd& vel);
	virtual bool UseWorldCoord(int joint_id) const;
	virtual void SetUseWorldCoord(int joint_id, bool use);
	virtual void SetKp(int joint_id, double kp);
	virtual void SetKd(int joint_id, double kd);
	virtual bool IsValidPDCtrl(int joint_id) const;

	virtual cPDController& GetPDCtrl(int joint_id);
	virtual const cPDController& GetPDCtrl(int joint_id) const;

protected:
	tEigenArr<cPDController> mPDCtrls;

	virtual void CalcControlForces(double time_step, Eigen::VectorXd& out_tau);
	virtual void ApplyControlForces(const Eigen::VectorXd& tau);
};