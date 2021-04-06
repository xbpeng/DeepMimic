#include "SceneTargetAMP.h"

double cSceneTargetAMP::CalcReward(int agent_id) const
{
	const double pos_reward_w = 0.6;
	const double vel_reward_w = 0.4;
	const bool enable_min_tar_vel = mEnableMinTarVel;

	double reward = 0.0;
	const cSimCharacter* sim_char = GetAgentChar(agent_id);
	bool dist_fail = CheckTarDistFail(sim_char->GetID());

	if (!dist_fail)
	{
		const double tar_speed = GetTargetSpeed();
		const double pos_err_scale = mPosRewardScale;
		const double vel_err_scale = 4 / (tar_speed * tar_speed);

		const auto& ctrl = GetController(agent_id);
		const cSimCharacter* character = GetAgentChar(agent_id);
		bool fallen = HasFallen(*character);
		if (!fallen)
		{
			const cDeepMimicCharController* char_ctrl = dynamic_cast<const cDeepMimicCharController*>(ctrl.get());
			double prev_action_time = char_ctrl->GetPrevActionTime();
			const tVector& prev_action_com = char_ctrl->GetPrevActionCOM();

			const tVector& tar_pos = GetTargetPos();
			tVector root_pos = character->GetRootPos();
			tVector root_tar_delta = tar_pos - root_pos;
			root_tar_delta[1] = 0.0;

			double root_tar_dist_sq = root_tar_delta.squaredNorm();
			double pos_reward = std::exp(-pos_err_scale * root_tar_dist_sq);

			double vel_reward = 0;
			const double dist_threshold = GetTargetSuccDist();

			if (root_tar_dist_sq < dist_threshold * dist_threshold)
			{
				vel_reward = 1.0;
			}
			else
			{
				double step_dur = char_ctrl->GetTime() - prev_action_time;

				tVector com = character->CalcCOM();
				tVector com_tar_delta = tar_pos - com;
				com_tar_delta[1] = 0.0;
				double com_tar_dist = com_tar_delta.norm();

				tVector com_tar_dir = tVector(0, 0, 0, 0);
				if (com_tar_dist > 0.0001)
				{
					com_tar_dir = com_tar_delta / com_tar_dist;
				}

				double avg_vel = com_tar_dir.dot(com - prev_action_com) / step_dur;
				double vel_err = tar_speed - avg_vel;

				if (avg_vel < 0)
				{
					vel_reward = 0.0;
				}
				else
				{
					if (enable_min_tar_vel)
					{
						vel_err = std::max(vel_err, 0.0);
					}
					vel_reward = std::exp(-vel_err_scale * vel_err * vel_err);
				}
			}

			reward = pos_reward_w * pos_reward + vel_reward_w * vel_reward;
		}
	}

	return reward;
}

cSceneTargetAMP::cSceneTargetAMP()
{
	mTargetTimerParams.mTimeMin = 1;
	mTargetTimerParams.mTimeMax = 5;
	mMaxTargetDist = 3;
	mTargetSuccDist = 0.5;
	mTarFailDist = std::numeric_limits<double>::infinity();

	mTargetPos.setZero();

	mTargetSpeed = 1;
	mEnableMinTarVel = false;
	mPosRewardScale = 1.0;

	EnableRandTargetPos(true);
}

cSceneTargetAMP::~cSceneTargetAMP()
{
}

void cSceneTargetAMP::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cSceneImitate::ParseArgs(parser);

	parser->ParseDouble("rand_target_time_min", mTargetTimerParams.mTimeMin);
	parser->ParseDouble("rand_target_time_max", mTargetTimerParams.mTimeMax);
	parser->ParseDouble("max_target_dist", mMaxTargetDist);
	parser->ParseDouble("target_succ_dist", mTargetSuccDist);
	parser->ParseDouble("tar_fail_dist", mTarFailDist);

	parser->ParseDouble("tar_speed", mTargetSpeed);
	parser->ParseBool("enable_min_tar_vel", mEnableMinTarVel);
	parser->ParseDouble("pos_reward_scale", mPosRewardScale);
}

void cSceneTargetAMP::Init()
{
	cSceneImitate::Init();
	InitTarget();
	ResetTarget();
}

void cSceneTargetAMP::Reset()
{
	cSceneImitate::Reset();
	mTargetTimer.Reset();
	ResetTarget();
}

void cSceneTargetAMP::Update(double time_elapsed)
{
	cSceneImitate::Update(time_elapsed);

	UpdateTarget(time_elapsed);
	if (mTargetTimer.IsEnd())
	{
		mTargetTimer.Reset();
	}
}

double cSceneTargetAMP::GetTargetSpeed() const
{
	return mTargetSpeed;
}

void cSceneTargetAMP::SetTargetSpeed(double speed)
{
	mTargetSpeed = speed;
}

tVector cSceneTargetAMP::GetTargetPos() const
{
	return mTargetPos;
}

void cSceneTargetAMP::SetTargetPos(const tVector& target_pos)
{
	mTargetPos = target_pos;
}

void cSceneTargetAMP::EnableRandTargetPos(bool enable)
{
	mEnableRandTargetPos = enable;
}

bool cSceneTargetAMP::EnableRandTargetPos() const
{
	return mEnableRandTargetPos;
}

double cSceneTargetAMP::GetTargetSuccDist() const
{
	return mTargetSuccDist;
}

bool cSceneTargetAMP::CheckTargetSucc() const
{
	const auto& character = GetCharacter();
	tVector root_pos = character->GetRootPos();
	tVector tar_pos = GetTargetPos();
	root_pos[1] = 0;
	tar_pos[1] = 0;
	double dist = (tar_pos - root_pos).norm();
	const double dist_threshold = GetTargetSuccDist();

	bool succ = (dist < dist_threshold);
	return succ;
}

void cSceneTargetAMP::RecordGoal(int agent_id, Eigen::VectorXd& out_goal) const
{
	int g_size = GetGoalSize(agent_id);
	out_goal.resize(g_size);

	const auto& sim_char = GetAgentChar(agent_id);
	tVector root_pos = sim_char->GetRootPos();
	tVector tar_pos = GetTargetPos();
	tMatrix origin_trans = sim_char->BuildOriginTrans();

	tVector rel_tar_pos = tar_pos - root_pos;
	rel_tar_pos[1] = 0;

	double tar_dist = rel_tar_pos.norm();
	if (tar_dist > 0.0001)
	{
		rel_tar_pos = origin_trans * rel_tar_pos;
		rel_tar_pos /= tar_dist;
	}
	else
	{
		rel_tar_pos = tVector(1, 0, 0, 0);
	}

	out_goal[0] = rel_tar_pos[0];
	out_goal[1] = rel_tar_pos[2];
	out_goal[2] = tar_dist;
}

int cSceneTargetAMP::GetGoalSize(int agent_id) const
{
	return 3;
}

bool cSceneTargetAMP::EnableAMPTaskReward() const
{
	return true;
}

std::string cSceneTargetAMP::GetName() const
{
	return "Target AMP";
}

void cSceneTargetAMP::InitTarget()
{
	mTargetTimer.Init(mTargetTimerParams);
}

void cSceneTargetAMP::UpdateTarget(double timestep)
{
	mTargetTimer.Update(timestep);
	
	bool reset_target = false;
	if (EnableRandTargetPos())
	{
		reset_target = CheckTargetReset(timestep);
	}

	if (reset_target)
	{
		ResetTargetPos();
	}
}

void cSceneTargetAMP::ResetTarget()
{
	ResetTargetPos();
}

bool cSceneTargetAMP::CheckTargetReset(double timestep) const
{
	bool reset = mTargetTimer.IsEnd();
	return reset;
}

void cSceneTargetAMP::ResetTargetPos()
{
	tVector rand_pos = SampleRandTargetPos();
	SetTargetPos(rand_pos);
}

tVector cSceneTargetAMP::SampleRandTargetPos()
{
	const double max_dist = GetMaxTargetDist();
	const auto& character = GetCharacter();
	tVector target_pos = tVector::Zero();
	tVector root_pos = character->GetRootPos();

	double dist = mRand.RandDouble(0.0, max_dist);
	double theta = mRand.RandDouble(0, 2 * M_PI);
	target_pos[0] = root_pos[0] + dist * std::cos(theta);
	target_pos[2] = root_pos[2] + dist * std::sin(theta);

	return target_pos;
}

double cSceneTargetAMP::GetMaxTargetDist() const
{
	return mMaxTargetDist;
}

bool cSceneTargetAMP::CheckTarDistFail(int char_id) const
{
	const auto& sim_char = GetCharacter(char_id);
	tVector char_pos = sim_char->GetRootPos();
	tVector pos_delta = char_pos - GetTargetPos();
	pos_delta[1] = 0;

	double distsq = pos_delta.squaredNorm();
	bool dist_fail = distsq > mTarFailDist * mTarFailDist;

	return dist_fail;
}

cSceneTargetAMP::eTerminate cSceneTargetAMP::CheckTerminate(int agent_id) const
{
	eTerminate terminated = cSceneImitateAMP::CheckTerminate(agent_id);

	if (terminated == eTerminate::eTerminateNull)
	{
		terminated = CheckTerminateTarget(agent_id);
	}

	return terminated;
}

cSceneTargetAMP::eTerminate cSceneTargetAMP::CheckTerminateTarget(int agent_id) const
{
	const auto& agent_char = GetAgentChar(agent_id);
	bool dist_fail = CheckTarDistFail(agent_char->GetID());

	eTerminate terminated = eTerminateNull;
	if (dist_fail)
	{
		terminated = eTerminateFail;
	}

	return terminated;
}

bool cSceneTargetAMP::EnableTestTimeWarp() const
{
	return false;
}