#include "RLSceneSimChar.h"

cRLSceneSimChar::cRLSceneSimChar()
{
	mEnableFallEnd = true;
	mAnnealSamples = gInvalidIdx;
}

cRLSceneSimChar::~cRLSceneSimChar()
{
}

void cRLSceneSimChar::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cRLScene::ParseArgs(parser);
	cSceneSimChar::ParseArgs(parser);

	parser->ParseBool("enable_fall_end", mEnableFallEnd);
	parser->ParseInt("anneal_samples", mAnnealSamples);

	mTimerParamsEnd = mTimerParams;
	mArgParser->ParseDouble("time_end_lim_min", mTimerParamsEnd.mTimeMin);
	mArgParser->ParseDouble("time_end_lim_max", mTimerParamsEnd.mTimeMax);
	mArgParser->ParseDouble("time_end_lim_exp", mTimerParamsEnd.mTimeExp);
}

void cRLSceneSimChar::Init()
{
	cRLScene::Init();
	cSceneSimChar::Init();

	mAgentReg.Clear();
	RegisterAgents();
	mAgentReg.PrintInfo();

	SetupTimerAnnealer(mTimerAnnealer);
}

void cRLSceneSimChar::Clear()
{
	cRLScene::Clear();
	cSceneSimChar::Clear(); 
	
	mAgentReg.Clear();
}

int cRLSceneSimChar::GetNumAgents() const
{
	return mAgentReg.GetNumAgents();
}

bool cRLSceneSimChar::NeedNewAction(int agent_id) const
{
	const auto& ctrl = GetController(agent_id);
	return ctrl->NeedNewAction();
}

void cRLSceneSimChar::RecordState(int agent_id, Eigen::VectorXd& out_state) const
{
	const auto& ctrl = GetController(agent_id);
	ctrl->RecordState(out_state);
}

void cRLSceneSimChar::RecordGoal(int agent_id, Eigen::VectorXd& out_goal) const
{
	int goal_size = GetGoalSize(agent_id);
	out_goal = std::numeric_limits<double>::quiet_NaN() * Eigen::VectorXd::Ones(goal_size);
}

void cRLSceneSimChar::SetAction(int agent_id, const Eigen::VectorXd& action)
{
	const auto& ctrl = GetController(agent_id);
	ctrl->ApplyAction(action);
}

eActionSpace cRLSceneSimChar::GetActionSpace(int agent_id) const
{
	const auto& ctrl = GetController(agent_id);
	return ctrl->GetActionSpace();
}

int cRLSceneSimChar::GetStateSize(int agent_id) const
{
	const auto& ctrl = GetController(agent_id);
	return ctrl->GetStateSize();
}

int cRLSceneSimChar::GetGoalSize(int agent_id) const
{
	return 0;
}

int cRLSceneSimChar::GetActionSize(int agent_id) const
{
	const auto& ctrl = GetController(agent_id);
	return ctrl->GetActionSize();
}

int cRLSceneSimChar::GetNumActions(int agent_id) const
{
	const auto& ctrl = GetController(agent_id);
	return ctrl->GetNumActions();
}

void cRLSceneSimChar::BuildStateOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	const auto& ctrl = GetController(agent_id);
	ctrl->BuildStateOffsetScale(out_offset, out_scale);
}

void cRLSceneSimChar::BuildGoalOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int goal_size = GetGoalSize(agent_id);
	out_offset = Eigen::VectorXd::Zero(goal_size);
	out_scale = Eigen::VectorXd::Ones(goal_size);
}

void cRLSceneSimChar::BuildActionOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	const auto& ctrl = GetController(agent_id);
	ctrl->BuildActionOffsetScale(out_offset, out_scale);
}

void cRLSceneSimChar::BuildActionBounds(int agent_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	const auto& ctrl = GetController(agent_id);
	ctrl->BuildActionBounds(out_min, out_max);
}

void cRLSceneSimChar::BuildStateNormGroups(int agent_id, Eigen::VectorXi& out_groups) const
{
	const auto& ctrl = GetController(agent_id);
	ctrl->BuildStateNormGroups(out_groups);
}

void cRLSceneSimChar::BuildGoalNormGroups(int agent_id, Eigen::VectorXi& out_groups) const
{
	int goal_size = GetGoalSize(agent_id);
	out_groups = cCharController::gNormGroupSingle * Eigen::VectorXi::Ones(goal_size);
}

double cRLSceneSimChar::GetRewardMin(int agent_id) const
{
	return mAgentReg.GetAgent(agent_id)->GetRewardMin();
}

double cRLSceneSimChar::GetRewardMax(int agent_id) const
{
	return mAgentReg.GetAgent(agent_id)->GetRewardMax();
}

bool cRLSceneSimChar::EnableAMPTaskReward() const
{
	return false;
}

int cRLSceneSimChar::GetAMPObsSize() const
{
	return 0;
}

void cRLSceneSimChar::GetAMPObsOffset(Eigen::VectorXd& out_data) const
{
	out_data.resize(0);
}

void cRLSceneSimChar::GetAMPObsScale(Eigen::VectorXd& out_data) const
{
	out_data.resize(0);
}

void cRLSceneSimChar::GetAMPObsNormGroup(Eigen::VectorXi& out_data) const
{
	out_data.resize(0);
}

void cRLSceneSimChar::RecordAMPObsAgent(int agent_id, Eigen::VectorXd& out_data)
{
	out_data.resize(0);
}

void cRLSceneSimChar::RecordAMPObsExpert(int agent_id, Eigen::VectorXd& out_data)
{
	out_data.resize(0);
}

cRLSceneSimChar::eTerminate cRLSceneSimChar::CheckTerminate(int agent_id) const
{
	bool fail = false;
	if (EnableFallEnd())
	{
		const auto& character = GetAgentChar(agent_id);
		fail |= HasFallen(*character);
	}
	eTerminate terminated = (fail) ? eTerminateFail : eTerminateNull;
	return terminated;
}

bool cRLSceneSimChar::CheckValidEpisode() const
{
	for (int i = 0; i < GetNumChars(); ++i)
	{
		const auto& sim_char = GetCharacter(i);
		bool exp = sim_char->HasVelExploded();
		if (exp)
		{
			return false;
		}
	}
	return true;
}

void cRLSceneSimChar::LogVal(int agent_id, double val)
{
	const auto& ctrl = GetController(agent_id);
	cDeepMimicCharController* char_ctrl = dynamic_cast<cDeepMimicCharController*>(ctrl.get());
	if (char_ctrl != nullptr)
	{
		char_ctrl->LogVal(val);
	}
}

void cRLSceneSimChar::SetSampleCount(int count)
{
	cRLScene::SetSampleCount(count);
	UpdateTimerParams();
}

std::string cRLSceneSimChar::GetName() const
{
	return "RL Sim Character";
}

void cRLSceneSimChar::ResetParams()
{
	cRLScene::ResetParams();
	cSceneSimChar::ResetParams();
}

void cRLSceneSimChar::ResetScene()
{
	cRLScene::ResetScene();
	cSceneSimChar::ResetScene();
}

const std::shared_ptr<cCharController>& cRLSceneSimChar::GetController() const
{
	const auto& character = GetCharacter();
	return character->GetController();
}

const std::shared_ptr<cCharController>& cRLSceneSimChar::GetController(int agent_id) const
{
	return mAgentReg.GetAgent(agent_id);
}

const cSimCharacter* cRLSceneSimChar::GetAgentChar(int agent_id) const
{
	return mAgentReg.GetChar(agent_id);
}

void cRLSceneSimChar::PreUpdate(double timestep)
{
	cSceneSimChar::PreUpdate(timestep);

	for (int a = 0; a < GetNumAgents(); ++a)
	{
		const auto& ctrl = mAgentReg.GetAgent(a);
		bool new_action = ctrl->NeedNewAction();
		if (new_action)
		{
			NewActionUpdate(a);
		}
	}
}

void cRLSceneSimChar::ResetTimers()
{
	cSceneSimChar::ResetTimers();
	if (mMode == eModeTest)
	{
		mTimer.SetMaxTime(mTimerParamsEnd.mTimeMax);
	}
}

void cRLSceneSimChar::NewActionUpdate(int agent_id)
{
}

bool cRLSceneSimChar::EnableFallEnd() const
{
	return mEnableFallEnd;
}

void cRLSceneSimChar::RegisterAgents()
{
	int num_chars = GetNumChars();
	for (int i = 0; i < num_chars; ++i)
	{
		const auto& character = GetCharacter(i);
		const auto& ctrl = character->GetController();
		RegisterAgent(ctrl, character);
	}
}

void cRLSceneSimChar::RegisterAgent(const std::shared_ptr<cCharController>& ctrl, const std::shared_ptr<cSimCharacter>& character)
{
	std::vector<int> ids;
	RegisterAgent(ctrl, character, ids);
}

void cRLSceneSimChar::RegisterAgent(const std::shared_ptr<cCharController>& ctrl,
									const std::shared_ptr<cSimCharacter>& character,
									std::vector<int>& out_ids)
{
	if (ctrl != nullptr)
	{
		int id = mAgentReg.AddAgent(ctrl, character.get());
		out_ids.push_back(id);

		int num_ctrls = ctrl->NumChildren();
		for (int i = 0; i < num_ctrls; ++i)
		{
			const auto& child_ctrl = ctrl->GetChild(i);
			RegisterAgent(child_ctrl, character, out_ids);
		}
	}
}

void cRLSceneSimChar::SetupTimerAnnealer(cAnnealer& out_annealer) const
{
	cAnnealer::tParams params;
	params.mType = cAnnealer::eTypePow;
	params.mPow = 4.0;
	out_annealer.Init(params);
}

void cRLSceneSimChar::UpdateTimerParams()
{
	if (mAnnealSamples > 0)
	{
		double t = static_cast<double>(mSampleCount) / mAnnealSamples;
		double lerp = mTimerAnnealer.Eval(t);
		cTimer::tParams blend_params = mTimerParams.Blend(mTimerParamsEnd, lerp);
		mTimer.SetParams(blend_params);
	}
}