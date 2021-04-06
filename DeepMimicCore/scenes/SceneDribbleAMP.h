#pragma once

#include "SceneTargetAMP.h"

class cSceneDribbleAMP : virtual public cSceneTargetAMP
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cSceneDribbleAMP();
	virtual ~cSceneDribbleAMP();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Init();
	virtual void Reset();

	virtual double CalcReward(int agent_id) const;
	virtual void SetBallPos(const tVector& pos);
	virtual tVector GetBallPos() const;
	virtual int GetTarObjID() const;

	virtual void RecordState(int agent_id, Eigen::VectorXd& out_state) const;
	virtual int GetStateSize(int agent_id) const;
	virtual void BuildStateOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildStateNormGroups(int agent_id, Eigen::VectorXi& out_groups) const;

	virtual void RecordGoal(int agent_id, Eigen::VectorXd& out_goal) const;

	virtual std::string GetName() const;

protected:

	static const double gComBallDistThreshold;

	double mMinTarObjDist;
	double mMaxTarObjDist;
	int mTarObjID;
	double mBallRadius;

	tVectorArr mAgentPrevTarObjPos;

	cTimer::tParams mTarObjTimerParams;
	cTimer mTarObjTimer;

	virtual void InitAgentTarObjRecord();
	virtual void ResetAgentTarObjRecord();
	virtual void UpdateAgentTarObjRecord(int agent_id);
	virtual double GetBallRadius() const;

	virtual void NewActionUpdate(int agent_id);
	virtual bool HasFallen(const cSimCharacter& sim_char) const;
	virtual bool CheckTarObjDistFail() const;
	virtual bool CheckCharObjDistFail(const cSimCharacter& sim_char) const;

	virtual void UpdateObjs(double timestep);
	virtual void ClearObjs();

	virtual double CalcRewardTrain(int agent_id) const;
	virtual double CalcRewardTest(int agent_id) const;

	virtual void InitTarObjs();
	virtual void BuildTarObjs();
	virtual void ResetTarObjs();
	virtual void UpdateTarObjs(double timestep);
	virtual bool CheckTargetSucc() const;
	virtual eTerminate CheckTerminateTarget(int agent_id) const;

	virtual double GetTarObjMinDist() const;
	virtual double GetTarObjMaxDist() const;
	virtual void SetTarObjPos(int obj_id, const tVector& pos);

	virtual tVector SampleRandTargetPos();

	virtual void RecordCtrlState(int agent_id, Eigen::VectorXd& out_state) const;
	virtual int GetCtrlStateSize(int agent_id) const;
	virtual void BuildCtrlStateOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildCtrlStateNormGroups(int agent_id, Eigen::VectorXi& out_groups) const;

	virtual void RecordTaskState(int agent_id, Eigen::VectorXd& out_state) const;
	virtual int GetTaskStateSize(int agent_id) const;
	virtual void BuildTaskStateOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildTaskStateNormGroups(int agent_id, Eigen::VectorXi& out_groups) const;
};