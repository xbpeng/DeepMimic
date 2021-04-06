#pragma once

#include "util/MathUtil.h"
#include "Scene.h"
#include "sim/ActionSpace.h"
#include "util/Timer.h"

class cRLScene : virtual public cScene
{
public:
	enum eMode
	{
		eModeTrain,
		eModeTest,
		eModeMax
	};

	enum eTerminate
	{
		eTerminateNull,
		eTerminateFail,
		eTerminateSucc,
		eTerminateMax
	};

	virtual ~cRLScene();
	
	virtual void Init();
	virtual void Clear();

	virtual int GetNumAgents() const = 0;
	virtual bool NeedNewAction(int agent_id) const = 0;
	virtual void RecordState(int agent_id, Eigen::VectorXd& out_state) const = 0;
	virtual void RecordGoal(int agent_id, Eigen::VectorXd& out_goal) const = 0;
	virtual void SetAction(int agent_id, const Eigen::VectorXd& action) = 0;

	virtual eActionSpace GetActionSpace(int agent_id) const = 0;
	virtual int GetStateSize(int agent_id) const = 0;
	virtual int GetGoalSize(int agent_id) const = 0;
	virtual int GetActionSize(int agent_id) const = 0;
	virtual int GetNumActions(int agent_id) const = 0;

	virtual void BuildStateOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const = 0;
	virtual void BuildGoalOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const = 0;
	virtual void BuildActionOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const = 0;
	virtual void BuildActionBounds(int agent_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const = 0;

	virtual void BuildStateNormGroups(int agent_id, Eigen::VectorXi& out_groups) const = 0;
	virtual void BuildGoalNormGroups(int agent_id, Eigen::VectorXi& out_groups) const = 0;

	virtual double CalcReward(int agent_id) const = 0;
	virtual double GetRewardMin(int agent_id) const = 0;
	virtual double GetRewardMax(int agent_id) const = 0;
	virtual double GetRewardFail(int agent_id);
	virtual double GetRewardSucc(int agent_id);

	virtual bool IsEpisodeEnd() const;
	virtual eTerminate CheckTerminate(int agent_id) const;
	virtual void SetSampleCount(int count);
	virtual void SetMode(eMode mode);

	virtual bool EnableAMPTaskReward() const = 0;
	virtual int GetAMPObsSize() const = 0;
	virtual void GetAMPObsOffset(Eigen::VectorXd& out_data) const = 0;
	virtual void GetAMPObsScale(Eigen::VectorXd& out_data) const = 0;
	virtual void GetAMPObsNormGroup(Eigen::VectorXi& out_data) const = 0;
	virtual void RecordAMPObsAgent(int agent_id, Eigen::VectorXd& out_data) = 0;
	virtual void RecordAMPObsExpert(int agent_id, Eigen::VectorXd& out_data) = 0;
	
	virtual void LogVal(int agent_id, double val) = 0;

protected:
	eMode mMode;
	int mSampleCount;

	cRLScene();
};