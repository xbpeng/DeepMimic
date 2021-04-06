#pragma once

#include "RLScene.h"
#include "SceneSimChar.h"
#include "sim/AgentRegistry.h"
#include "util/Annealer.h"

class cRLSceneSimChar : virtual public cRLScene, virtual public cSceneSimChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cRLSceneSimChar();
	virtual ~cRLSceneSimChar();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Init();
	virtual void Clear();

	virtual int GetNumAgents() const;
	virtual bool NeedNewAction(int agent_id) const;
	virtual void RecordState(int agent_id, Eigen::VectorXd& out_state) const;
	virtual void RecordGoal(int agent_id, Eigen::VectorXd& out_goal) const;
	virtual void SetAction(int agent_id, const Eigen::VectorXd& action);

	virtual eActionSpace GetActionSpace(int agent_id) const;
	virtual int GetStateSize(int agent_id) const;
	virtual int GetGoalSize(int agent_id) const;
	virtual int GetActionSize(int agent_id) const;
	virtual int GetNumActions(int agent_id) const;

	virtual void BuildStateOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildGoalOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildActionOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildActionBounds(int agent_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;

	virtual void BuildStateNormGroups(int agent_id, Eigen::VectorXi& out_groups) const;
	virtual void BuildGoalNormGroups(int agent_id, Eigen::VectorXi& out_groups) const;

	virtual double CalcReward(int agent_id) const = 0;
	virtual double GetRewardMin(int agent_id) const;
	virtual double GetRewardMax(int agent_id) const;

	virtual bool EnableAMPTaskReward() const;
	virtual int GetAMPObsSize() const;
	virtual void GetAMPObsOffset(Eigen::VectorXd& out_data) const;
	virtual void GetAMPObsScale(Eigen::VectorXd& out_data) const;
	virtual void GetAMPObsNormGroup(Eigen::VectorXi& out_data) const;
	virtual void RecordAMPObsAgent(int agent_id, Eigen::VectorXd& out_data);
	virtual void RecordAMPObsExpert(int agent_id, Eigen::VectorXd& out_data);
	
	virtual eTerminate CheckTerminate(int agent_id) const;
	virtual bool CheckValidEpisode() const;
	virtual void LogVal(int agent_id, double val);
	virtual void SetSampleCount(int count);

	virtual std::string GetName() const;

protected:

	bool mEnableFallEnd;
	cAgentRegistry mAgentReg;
	cTimer::tParams mTimerParamsEnd;
	cAnnealer mTimerAnnealer;
	int mAnnealSamples;

	virtual void ResetParams();
	virtual void ResetScene();
	virtual const std::shared_ptr<cCharController>& GetController() const;
	virtual const std::shared_ptr<cCharController>& GetController(int agent_id) const;
	virtual const cSimCharacter* GetAgentChar(int agent_id) const;

	virtual void PreUpdate(double timestep);
	virtual void ResetTimers();

	virtual void NewActionUpdate(int agent_id);
	virtual bool EnableFallEnd() const;

	virtual void RegisterAgents();
	virtual void RegisterAgent(const std::shared_ptr<cCharController>& ctrl, const std::shared_ptr<cSimCharacter>& character);
	virtual void RegisterAgent(const std::shared_ptr<cCharController>& ctrl, const std::shared_ptr<cSimCharacter>& character,
								std::vector<int>& out_ids);

	virtual void SetupTimerAnnealer(cAnnealer& out_annealer) const;
	virtual void UpdateTimerParams();
};