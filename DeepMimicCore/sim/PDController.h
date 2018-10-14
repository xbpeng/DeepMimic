#pragma once

#include "util/json/json.h"
#include "sim/Controller.h"
#include "sim/SimBodyJoint.h"

class cPDController : public cController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum eParam
	{
		eParamJointID,
		eParamKp,
		eParamKd,
		eParamTargetTheta0,
		eParamTargetTheta1,
		eParamTargetTheta2,
		eParamTargetTheta3,
		eParamTargetTheta4,
		eParamTargetTheta5,
		eParamTargetTheta6,
		eParamTargetVel0,
		eParamTargetVel1,
		eParamTargetVel2,
		eParamTargetVel3,
		eParamTargetVel4,
		eParamTargetVel5,
		eParamTargetVel6,
		eParamUseWorldCoord,
		eParamMax
	};
	typedef Eigen::Matrix<double, eParamMax, 1> tParams;

	static bool LoadParams(const std::string& file, Eigen::MatrixXd& out_buffer);
	static bool LoadParams(const Json::Value& root, Eigen::MatrixXd& out_buffer);
	static bool ParsePDParams(const Json::Value& root, tParams& out_params);

	cPDController();
	virtual ~cPDController();

	virtual void Init(cSimCharacter* character, const tParams& params);
	virtual void Clear();
	virtual void Update(double time_step);
	virtual void UpdateControlForce(double time_step, Eigen::VectorXd& out_tau);

	virtual cSimBodyJoint& GetJoint();
	virtual const cSimBodyJoint& GetJoint() const;

	virtual double GetKp() const;
	virtual void SetKp(double kp);
	virtual void SetKd(double kd);
	virtual double GetKd() const;
	virtual double GetTorqueLimit() const;
	virtual double GetForceLimit() const;
	virtual void SetTargetTheta(const Eigen::VectorXd& theta);
	virtual void SetTargetVel(const Eigen::VectorXd& vel);
	virtual void SetUseWorldCoord(bool use);
	virtual bool UseWorldCoord() const;

	virtual void GetTargetTheta(Eigen::VectorXd& out_theta) const;
	virtual void GetTargetVel(Eigen::VectorXd& out_vel) const;

	virtual bool IsActive() const;

protected:
	tParams mParams;
	
	virtual int GetJointDim() const;
	virtual int GetJointID() const;
	virtual void ApplyControlForces(const Eigen::VectorXd& tau);

	virtual void CalcJointTau(double time_step, Eigen::VectorXd& out_joint_tau);
	virtual void CalcJointTauRevolute(double time_step, Eigen::VectorXd& out_joint_tau);
	virtual void CalcJointTauPlanar(double time_step, Eigen::VectorXd& out_joint_tau);
	virtual void CalcJointTauPrismatic(double time_step, Eigen::VectorXd& out_joint_tau);
	virtual void CalcJointTauFixed(double time_step, Eigen::VectorXd& out_joint_tau);
	virtual void CalcJointTauSpherical(double time_step, Eigen::VectorXd& out_joint_tau);

	virtual void PostProcessTargetPose(Eigen::VectorXd& out_pose) const;
	virtual void PostProcessTargetVel(Eigen::VectorXd& out_vel) const;
};
