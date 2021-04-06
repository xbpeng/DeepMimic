#pragma once

#include "sim/DeepMimicCharController.h"
#include "sim/CtCtrlUtil.h"

class cCtController : public virtual cDeepMimicCharController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cCtController();
	virtual ~cCtController();

	virtual void SetUpdateRate(double rate);
	virtual double GetUpdateRate() const;

	virtual void UpdateCalcTau(double timestep, Eigen::VectorXd& out_tau);
	
	virtual double GetCyclePeriod() const;
	virtual void SetCyclePeriod(double period);
	virtual void SetInitTime(double time);
	virtual double GetPhase() const;

	virtual int GetStateSize() const;
	virtual int GetActionSize() const;
	virtual void BuildStateOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;
	virtual void BuildActionOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;

	virtual void BuildStateNormGroups(Eigen::VectorXi& out_groups) const;
	
	virtual void RecordState(Eigen::VectorXd& out_state);

	virtual std::string GetName() const;

protected:
	
	double mUpdateRate;
	double mCyclePeriod;
	bool mEnablePhaseInput;
	bool mRecordWorldRootPos;
	bool mRecordWorldRootRot;

	double mPhaseOffset;
	double mInitTimeOffset;

	int mActionSize;
	Eigen::VectorXi mCtrlParamOffset;

	virtual void ResetParams();
	virtual int GetPosFeatureDim() const;
	virtual int GetRotFeatureDim() const;
	virtual int GetVelFeatureDim() const;
	virtual int GetAngVelFeatureDim() const;

	virtual bool ParseParams(const Json::Value& json);
	virtual void InitResources();
	virtual void BuildCtrlParamOffset(Eigen::VectorXi& out_offset) const;

	virtual void UpdateBuildTau(double time_step, Eigen::VectorXd& out_tau);
	virtual bool CheckNeedNewAction(double timestep) const;
	virtual void NewActionUpdate();
	
	virtual int GetStatePoseSize() const;
	virtual int GetStateVelSize() const;
	virtual int GetStatePoseOffset() const;
	virtual int GetStatePhaseSize() const;
	virtual int GetStatePhaseOffset() const;

	virtual int GetActionCtrlOffset() const;
	virtual int GetActionCtrlSize() const;

	virtual int GetCtrlParamOffset(int joint_id) const;
	virtual int GetCtrlParamSize(int joint_id) const;
	
	virtual void BuildStatePhaseOffsetScale(Eigen::VectorXd& phase_offset, Eigen::VectorXd& phase_scale) const;

	virtual void BuildStatePose(Eigen::VectorXd& out_pose) const;
	virtual void BuildStateVel(Eigen::VectorXd& out_vel) const;
	virtual void BuildStatePhase(Eigen::VectorXd& out_phase) const;

	virtual void BuildJointActionBounds(int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;
	virtual void BuildJointActionOffsetScale(int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;

	virtual int RetargetJointID(int joint_id) const;
};