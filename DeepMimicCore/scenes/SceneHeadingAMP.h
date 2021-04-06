#pragma once

#include "SceneTargetAMP.h"

class cSceneHeadingAMP : virtual public cSceneTargetAMP
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cSceneHeadingAMP();
	virtual ~cSceneHeadingAMP();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual double CalcReward(int agent_id) const;

	virtual void SetTargetSpeed(double speed);
	virtual double GetTargetHeading() const;
	virtual void SetTargetHeading(double heading);
	virtual tVector GetTargetPos() const;
	virtual void SetTargetPos(const tVector& target_pos);
	virtual bool EnableTargetPos() const;
	virtual void EnableTargetPos(bool enable);
	virtual bool EnableRandSpeed() const;
	virtual void EnableRandSpeed(bool enable);

	virtual int GetGoalSize(int agent_id) const;
	virtual void RecordGoal(int agent_id, Eigen::VectorXd& out_goal) const;

	virtual std::string GetName() const;

protected:
	bool mEnableTargetPos;
	bool mEnableRandSpeed;

	double mMaxHeadingTurnRate;
	double mSharpTurnProb;
	double mSpeedChangeProb;

	double mTargetSpeedMin;
	double mTargetSpeedMax;
	double mTargetHeading;
	double mVelRewardScale;

	virtual void UpdateTargetHeading();
	virtual void UpdateTargetSpeed();
	virtual void UpdateTarget(double timestep);
	virtual void ResetTarget();

	virtual bool CheckTarDistFail(int char_id) const;
};