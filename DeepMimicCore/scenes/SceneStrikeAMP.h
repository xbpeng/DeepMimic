#pragma once

#include "scenes/SceneTargetAMP.h"

class cSceneStrikeAMP : virtual public cSceneTargetAMP
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cSceneStrikeAMP();
	virtual ~cSceneStrikeAMP();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Clear();

	virtual void SetTargetPosXZ(const tVector& target_pos);
	virtual void SetTargetHit(bool hit);
	virtual bool TargetHit() const;
	virtual double GetTargetRadius() const;
	virtual const std::vector<int> GetStrikeBodies() const;

	virtual int GetGoalSize(int agent_id) const;
	virtual void BuildGoalNormGroups(int agent_id, Eigen::VectorXi& out_groups) const;
	virtual void RecordGoal(int agent_id, Eigen::VectorXd& out_goal) const;

	virtual double CalcReward(int agent_id) const;
	virtual double GetRewardSucc(int agent_id);

	virtual double GetTarNearDist() const;

	virtual std::string GetName() const;

protected:

	tVector mTargetMin;
	tVector mTargetMax;
	double mTargetRadius;
	bool mTargetHit;
	double mTargetHitTime;
	double mTargetHitResetTime;

	double mTarRewardScale;
	double mHitTarSpeed;
	double mInitHitProb;

	double mTarFarProb;
	double mTarNearDist;

	std::vector<int> mStrikeBodies;
	std::vector<int> mFailTarContactBodies;

	virtual void UpdateTarget(double timestep);
	virtual void ResetTarget();
	virtual void ResetTargetPos();
	virtual void ResetTargetPosFar();
	virtual void ResetTargetPosNear();
	virtual void ResetTargetHit();
	virtual bool CheckTargetReset(double timestep) const;

	virtual double CalcHitPhase() const;

	virtual double CalcRewardTrain(int agent_id) const;
	virtual double CalcRewardTest(int agent_id) const;
	virtual double CalcRewardTargetNear(int agent_id) const;
	virtual double CalcRewardTargetFar(int agent_id) const;

	virtual int GetNumStrikeBodies() const;
	virtual bool CheckTargetHit() const;
	virtual bool CheckResetTargetDist() const;
	virtual bool CheckTarContactFail(int char_id) const;
	virtual bool CheckTarHitSucc() const;
	virtual eTerminate CheckTerminateTarget(int agent_id) const;
};