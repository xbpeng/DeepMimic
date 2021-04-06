#include "DrawRLScene.h"

cDrawRLScene::cDrawRLScene()
{
}

cDrawRLScene::~cDrawRLScene()
{
}

void cDrawRLScene::Init()
{
	cRLScene::Init();
	cDrawScene::Init();
}

int cDrawRLScene::GetNumAgents() const
{
	return GetRLScene()->GetNumAgents();
}

bool cDrawRLScene::NeedNewAction(int agent_id) const
{
	return GetRLScene()->NeedNewAction(agent_id);
}

void cDrawRLScene::RecordState(int agent_id, Eigen::VectorXd& out_state) const
{
	GetRLScene()->RecordState(agent_id, out_state);
}

void cDrawRLScene::RecordGoal(int agent_id, Eigen::VectorXd& out_goal) const
{
	GetRLScene()->RecordGoal(agent_id, out_goal);
}

void cDrawRLScene::SetAction(int agent_id, const Eigen::VectorXd& action)
{
	GetRLScene()->SetAction(agent_id, action);
}

eActionSpace cDrawRLScene::GetActionSpace(int agent_id) const
{
	return GetRLScene()->GetActionSpace(agent_id);
}

int cDrawRLScene::GetStateSize(int agent_id) const
{
	return GetRLScene()->GetStateSize(agent_id);
}

int cDrawRLScene::GetGoalSize(int agent_id) const
{
	return GetRLScene()->GetGoalSize(agent_id);
}

int cDrawRLScene::GetActionSize(int agent_id) const
{
	return GetRLScene()->GetActionSize(agent_id);
}

int cDrawRLScene::GetNumActions(int agent_id) const
{
	return GetRLScene()->GetNumActions(agent_id);
}

void cDrawRLScene::BuildStateOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	GetRLScene()->BuildStateOffsetScale(agent_id, out_offset, out_scale);
}

void cDrawRLScene::BuildGoalOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	GetRLScene()->BuildGoalOffsetScale(agent_id, out_offset, out_scale);
}

void cDrawRLScene::BuildActionOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	GetRLScene()->BuildActionOffsetScale(agent_id, out_offset, out_scale);
}

void cDrawRLScene::BuildActionBounds(int agent_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	GetRLScene()->BuildActionBounds(agent_id, out_min, out_max);
}

void cDrawRLScene::BuildStateNormGroups(int agent_id, Eigen::VectorXi& out_groups) const
{
	GetRLScene()->BuildStateNormGroups(agent_id, out_groups);
}

void cDrawRLScene::BuildGoalNormGroups(int agent_id, Eigen::VectorXi& out_groups) const
{
	GetRLScene()->BuildGoalNormGroups(agent_id, out_groups);
}

double cDrawRLScene::CalcReward(int agent_id) const
{
	return GetRLScene()->CalcReward(agent_id);
}

double cDrawRLScene::GetRewardMin(int agent_id) const
{
	return GetRLScene()->GetRewardMin(agent_id);
}

double cDrawRLScene::GetRewardMax(int agent_id) const
{
	return GetRLScene()->GetRewardMax(agent_id);
}

double cDrawRLScene::GetRewardFail(int agent_id)
{
	return GetRLScene()->GetRewardFail(agent_id);
}

double cDrawRLScene::GetRewardSucc(int agent_id)
{
	return GetRLScene()->GetRewardSucc(agent_id);
}

bool cDrawRLScene::IsEpisodeEnd() const
{
	return GetRLScene()->IsEpisodeEnd();
}

cDrawRLScene::eTerminate cDrawRLScene::CheckTerminate(int agent_id) const
{
	return GetRLScene()->CheckTerminate(agent_id);
}

bool cDrawRLScene::CheckValidEpisode() const
{
	return GetRLScene()->CheckValidEpisode();
}

void cDrawRLScene::SetMode(eMode mode)
{
	return GetRLScene()->SetMode(mode);
}

bool cDrawRLScene::EnableAMPTaskReward() const
{
	return GetRLScene()->EnableAMPTaskReward();
}

int cDrawRLScene::GetAMPObsSize() const
{
	return GetRLScene()->GetAMPObsSize();
}

void cDrawRLScene::GetAMPObsOffset(Eigen::VectorXd& out_data) const
{
	GetRLScene()->GetAMPObsOffset(out_data);
}

void cDrawRLScene::GetAMPObsScale(Eigen::VectorXd& out_data) const
{
	GetRLScene()->GetAMPObsScale(out_data);
}

void cDrawRLScene::GetAMPObsNormGroup(Eigen::VectorXi& out_data) const
{
	GetRLScene()->GetAMPObsNormGroup(out_data);
}

void cDrawRLScene::RecordAMPObsAgent(int agent_id, Eigen::VectorXd& out_data)
{
	GetRLScene()->RecordAMPObsAgent(agent_id, out_data);
}

void cDrawRLScene::RecordAMPObsExpert(int agent_id, Eigen::VectorXd& out_data)
{
	GetRLScene()->RecordAMPObsExpert(agent_id, out_data);
}


void cDrawRLScene::SetSampleCount(int count)
{
	return GetRLScene()->SetSampleCount(count);
}

void cDrawRLScene::LogVal(int agent_id, double val)
{
	GetRLScene()->LogVal(agent_id, val);
}

std::string cDrawRLScene::GetName() const
{
	return GetRLScene()->GetName();
}