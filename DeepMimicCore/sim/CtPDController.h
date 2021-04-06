#pragma once

#include "sim/CtController.h"
#include "sim/ImpPDController.h"

class cCtPDController : public virtual cCtController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cCtPDController();
	virtual ~cCtPDController();

	virtual void Reset();
	virtual void Clear();

	virtual void SetGravity(const tVector& gravity);

	virtual std::string GetName() const;

protected:
	cImpPDController mPDCtrl;

	tVector mGravity;

	virtual bool ParseParams(const Json::Value& json);
	virtual int GetCtrlParamSize(int joint_id) const;
	virtual double GetMaxPDExpVal() const;

	virtual void UpdateBuildTau(double time_step, Eigen::VectorXd& out_tau);
	virtual void SetupPDControllers(const Json::Value& json, const tVector& gravity);
	virtual void UpdatePDCtrls(double time_step, Eigen::VectorXd& out_tau);
	virtual void ApplyAction(const Eigen::VectorXd& action);
	virtual void BuildJointActionBounds(int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;
	virtual void BuildJointActionOffsetScale(int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void ConvertActionToTargetPose(int joint_id, const Eigen::VectorXd& action_params, Eigen::VectorXd& out_theta) const;
	virtual cKinTree::eJointType GetJointType(int joint_id) const;

	virtual void SetPDTargets(const Eigen::VectorXd& targets);
};