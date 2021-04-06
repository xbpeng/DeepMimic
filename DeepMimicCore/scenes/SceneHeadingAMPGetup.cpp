#include "SceneHeadingAMPGetup.h"
#include "anim/ClipsController.h"

double cSceneHeadingAMPGetup::CalcReward(int agent_id) const
{
	double r = 0.0;
	bool getting_up = CheckGettingUp();
	if (getting_up)
	{
		r = CalcRewardGetup(agent_id);
	}
	else
	{
		r = cSceneHeadingAMP::CalcReward(agent_id);
	}
	return r;
}

double cSceneHeadingAMPGetup::CalcRewardGetup(int agent_id) const
{
	const auto* sim_char = GetAgentChar(agent_id);
	tVector root_pos = sim_char->GetRootPos();
	tVector head_pos = sim_char->GetBodyPartPos(mHeadID);
	const auto& ground = GetGround();
	double ground_h = ground->SampleHeight(root_pos);

	double root_h = root_pos[1] - ground_h;
	double norm_root_h = root_h / mGetupHeightRoot;
	norm_root_h = cMathUtil::Clamp(norm_root_h, 0.0, 1.0);

	double head_h = head_pos[1] - ground_h;
	double norm_head_h = head_h / mGetupHeightHead;
	norm_head_h = cMathUtil::Clamp(norm_head_h, 0.0, 1.0);

	double r = 0.2 * norm_root_h + 0.8 * norm_head_h;

	return r;
}

void cSceneHeadingAMPGetup::ResetRecoveryEpisode()
{
	ResetTimers();
	BeginGetup();

	int num_chars = GetNumChars();
	for (int c = 0; c < num_chars; ++c)
	{
		const auto& sim_char = GetCharacter(c);
		const auto& ctrl = sim_char->GetController();
		if (ctrl != nullptr)
		{
			ctrl->Reset();
		}
	}
}

cSceneHeadingAMPGetup::cSceneHeadingAMPGetup() : cSceneHeadingAMP()
{
	mGetupTime = 0.0;
	mGetupHeightRoot = 0.5;
	mGetupHeightHead = 0.5;
	mHeadID = 0;

	mRecoverEpisodeProb = 0.0;
	mIsRecoveryEpisode = false;
}

cSceneHeadingAMPGetup::~cSceneHeadingAMPGetup()
{
}

void cSceneHeadingAMPGetup::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cSceneHeadingAMP::ParseArgs(parser);

	parser->ParseInts("getup_motion_ids", mGetupMotionIDs);
	parser->ParseDouble("getup_height_root", mGetupHeightRoot);
	parser->ParseDouble("getup_height_head", mGetupHeightHead);
	parser->ParseInt("head_id", mHeadID);
	parser->ParseDouble("recover_episode_prob", mRecoverEpisodeProb);
}

void cSceneHeadingAMPGetup::Init()
{
	cSceneHeadingAMP::Init();

	RecordGetupMotionFlags(mGetupMotionIDs);
	mGetupTime = CalcGetupTime(mGetupMotionIDs);

	InitGetupTimer();
	ResetGetupTimer();
	SyncGetupTimer();
}

void cSceneHeadingAMPGetup::Update(double timestep)
{
	cSceneHeadingAMP::Update(timestep);

	if (mMode == eModeTest)
	{
		UpdateTestGetup();
	}
}

void cSceneHeadingAMPGetup::Reset()
{
	bool mIsRecoveryEpisode = ActivateRecoveryEpisode();
	if (mIsRecoveryEpisode)
	{
		ResetRecoveryEpisode();
	}
	else
	{
		cSceneHeadingAMP::Reset();
		SyncGetupTimer();
	}
}

void cSceneHeadingAMPGetup::RecordGoal(int agent_id, Eigen::VectorXd& out_goal) const
{
	cSceneHeadingAMP::RecordGoal(agent_id, out_goal);

	double getup_phase = CalcGetupPhase();
	int offset = cSceneHeadingAMP::GetGoalSize(agent_id);
	out_goal[offset] = getup_phase;
}

int cSceneHeadingAMPGetup::GetGoalSize(int agent_id) const
{
	int g_size = cSceneHeadingAMP::GetGoalSize(agent_id);
	g_size += 1;
	return g_size;
}

void cSceneHeadingAMPGetup::BuildGoalOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	cSceneHeadingAMP::BuildGoalOffsetScale(agent_id, out_offset, out_scale);

	int offset = cSceneHeadingAMP::GetGoalSize(agent_id);
	out_offset[offset] = -0.5;
	out_scale[offset] = 2.0;
}

void cSceneHeadingAMPGetup::BuildGoalNormGroups(int agent_id, Eigen::VectorXi& out_groups) const
{
	cSceneHeadingAMP::BuildGoalNormGroups(agent_id, out_groups);

	int offset = cSceneHeadingAMP::GetGoalSize(agent_id);
	out_groups[offset] = cCharController::gNormGroupNone;
}

std::string cSceneHeadingAMPGetup::GetName() const
{
	return "Heading AMP Getup";
}

void cSceneHeadingAMPGetup::ResetTimers()
{
	cSceneHeadingAMP::ResetTimers();
	ResetGetupTimer();
}

void cSceneHeadingAMPGetup::UpdateTimers(double timestep)
{
	cSceneHeadingAMP::UpdateTimers(timestep);
	UpdateGetupTimer(timestep);
}

void cSceneHeadingAMPGetup::InitGetupTimer()
{
	mIsRecoveryEpisode = false;

	cTimer::tParams getup_timer_params;
	getup_timer_params.mType = cTimer::eTypeUniform;
	getup_timer_params.mTimeMin = mGetupTime;
	getup_timer_params.mTimeMax = mGetupTime;

	mGetupTimer.Init(getup_timer_params);
}

void cSceneHeadingAMPGetup::ResetGetupTimer()
{
	mGetupTimer.Reset();
	EndGetup();
}

void cSceneHeadingAMPGetup::SyncGetupTimer()
{
	const auto& kin_char = GetKinChar();
	const auto kin_ctrl = kin_char->GetController();
	const cClipsController* clips_ctrl = dynamic_cast<const cClipsController*>(kin_ctrl.get());
	if (clips_ctrl != nullptr)
	{
		int curr_motion_id = clips_ctrl->GetCurrMotionID();
		bool is_getup_motion = mGetupMotionFlags[curr_motion_id];

		if (is_getup_motion)
		{
			double kin_time = clips_ctrl->GetTime();
			mGetupTimer.SetTime(kin_time);
		}
	}
}

void cSceneHeadingAMPGetup::UpdateGetupTimer(double timestep)
{
	mGetupTimer.Update(timestep);
}

void cSceneHeadingAMPGetup::RecordGetupMotionFlags(const std::vector<int>& getup_motion_ids)
{
	const auto& kin_char = GetKinChar();
	const auto kin_ctrl = kin_char->GetController();
	const cClipsController* clips_ctrl = dynamic_cast<const cClipsController*>(kin_ctrl.get());

	if (clips_ctrl != nullptr)
	{
		int num_motions = clips_ctrl->GetNumMotions();
		mGetupMotionFlags.resize(num_motions, false);

		int num_getup_motions = static_cast<int>(getup_motion_ids.size());
		for (int i = 0; i < num_getup_motions; ++i)
		{
			int curr_id = getup_motion_ids[i];
			mGetupMotionFlags[curr_id] = true;
		}
	}
	else
	{
		assert(false), "Unsupported kin motion controller.";
	}
}

void cSceneHeadingAMPGetup::BeginGetup()
{
	mGetupTimer.SetTime(0.0);
}

void cSceneHeadingAMPGetup::EndGetup()
{
	mGetupTimer.SetTime(mGetupTime);
}

void cSceneHeadingAMPGetup::UpdateTestGetup()
{
	const auto& sim_char = GetCharacter();
	bool fallen = HasFallenContact(*sim_char);
	bool is_getup = CheckGettingUp();
	if (fallen && !is_getup)
	{
		BeginGetup();
	}
}

bool cSceneHeadingAMPGetup::HasFallenContact(const cSimCharacter& sim_char) const
{
	bool fallen = false;
	bool getting_up = CheckGettingUp();
	if (!getting_up)
	{
		fallen = cSceneHeadingAMP::HasFallenContact(sim_char);
	}
	return fallen;
}

double cSceneHeadingAMPGetup::CalcGetupTime(const std::vector<int>& getup_motion_ids) const
{
	double getup_time = 0.0;
	const auto& kin_char = GetKinChar();

	const auto kin_ctrl = kin_char->GetController();
	const cClipsController* clips_ctrl = dynamic_cast<const cClipsController*>(kin_ctrl.get());

	if (clips_ctrl != nullptr)
	{
		int num_getup_motions = static_cast<int>(getup_motion_ids.size());
		for (int i = 0; i < num_getup_motions; ++i)
		{
			int motion_id = getup_motion_ids[i];
			const cMotion& curr_motion = clips_ctrl->GetMotion(motion_id);
			double curr_dur = curr_motion.GetDuration();
			getup_time = std::max(curr_dur, getup_time);
		}
	}
	else
	{
		assert(false), "Unsupported kin motion controller.";
	}
	return getup_time;
}

double cSceneHeadingAMPGetup::CalcGetupPhase() const
{
	double getup_time = mGetupTimer.GetTime();
	double phase = 1.0 - getup_time / mGetupTime;
	phase = cMathUtil::Clamp(phase, 0.0, 1.0);
	return phase;
}

bool cSceneHeadingAMPGetup::CheckGettingUp() const
{
	return !mGetupTimer.IsEnd();
}

bool cSceneHeadingAMPGetup::ActivateRecoveryEpisode()
{
	bool activate = false;

	if ((mMode == eModeTrain) && (mRecoverEpisodeProb > 0.0) && !mIsRecoveryEpisode)
	{
		eTerminate terminate = CheckTerminate(0);
		if (terminate == eTerminateFail)
		{
			activate = mRand.FlipCoin(mRecoverEpisodeProb);
		}
	}
	return activate;
}