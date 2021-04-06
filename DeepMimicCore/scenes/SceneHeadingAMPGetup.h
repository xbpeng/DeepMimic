#pragma once

#include "scenes/SceneHeadingAMP.h"

class cSceneHeadingAMPGetup : virtual public cSceneHeadingAMP
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cSceneHeadingAMPGetup();
	virtual ~cSceneHeadingAMPGetup();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Init();
	virtual void Update(double timestep);
	virtual void Reset();

	virtual double CalcReward(int agent_id) const;

	virtual void RecordGoal(int agent_id, Eigen::VectorXd& out_goal) const;
	virtual int GetGoalSize(int agent_id) const;
	virtual void BuildGoalOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildGoalNormGroups(int agent_id, Eigen::VectorXi& out_groups) const;

	virtual std::string GetName() const;

protected:
	std::vector<int> mGetupMotionIDs;
	std::vector<bool> mGetupMotionFlags;

	double mGetupTime;
	double mGetupHeightRoot;
	double mGetupHeightHead;
	int mHeadID;

	double mRecoverEpisodeProb;
	bool mIsRecoveryEpisode;

	cTimer mGetupTimer;

	virtual void ResetTimers();
	virtual void UpdateTimers(double timestep);
	virtual void InitGetupTimer();
	virtual void ResetGetupTimer();
	virtual void SyncGetupTimer();
	virtual void UpdateGetupTimer(double timestep);

	virtual void RecordGetupMotionFlags(const std::vector<int>& getup_motion_ids);
	virtual void BeginGetup();
	virtual void EndGetup();
	virtual void UpdateTestGetup();

	virtual bool HasFallenContact(const cSimCharacter& sim_char) const;
	virtual double CalcGetupTime(const std::vector<int>& getup_motion_ids) const;
	virtual double CalcRewardGetup(int agent_id) const;
	virtual void ResetRecoveryEpisode();

	virtual double CalcGetupPhase() const;
	virtual bool CheckGettingUp() const;
	virtual bool ActivateRecoveryEpisode();
};