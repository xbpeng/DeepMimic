#pragma once

#include "SceneImitateAMP.h"

class cSceneTargetAMP : virtual public cSceneImitateAMP
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cSceneTargetAMP();
	virtual ~cSceneTargetAMP();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual double CalcReward(int agent_id) const;

	virtual void Init();
	virtual void Reset();
	virtual void Update(double time_elapsed);

	virtual double GetTargetSpeed() const;
	virtual void SetTargetSpeed(double speed);
	virtual tVector GetTargetPos() const;
	virtual void SetTargetPos(const tVector& target_pos);
	virtual void EnableRandTargetPos(bool enable);
	virtual bool EnableRandTargetPos() const;
	virtual double GetTargetSuccDist() const;
	virtual bool CheckTargetSucc() const;
	virtual eTerminate CheckTerminate(int agent_id) const;

	virtual void RecordGoal(int agent_id, Eigen::VectorXd& out_goal) const;
	virtual int GetGoalSize(int agent_id) const;
	virtual bool EnableAMPTaskReward() const;

	virtual std::string GetName() const;

protected:

	bool mEnableRandTargetPos;
	double mMaxTargetDist;
	double mTargetSuccDist;
	double mTarFailDist;

	cTimer::tParams mTargetTimerParams;
	cTimer mTargetTimer;
	tVector mTargetPos;

	double mTargetSpeed;
	bool mEnableMinTarVel;
	double mPosRewardScale;

	virtual void InitTarget();
	virtual void UpdateTarget(double timestep);
	virtual void ResetTarget();

	virtual bool CheckTargetReset(double timestep) const;
	virtual void ResetTargetPos();
	virtual tVector SampleRandTargetPos();
	virtual double GetMaxTargetDist() const;
	virtual bool CheckTarDistFail(int char_id) const;
	virtual eTerminate CheckTerminateTarget(int agent_id) const;

	virtual bool EnableTestTimeWarp() const;
};