#include "SceneStrikeAMP.h"

const double gInvalidHitTime = -1.0;

const double gTarFarRewardWeight = 0.3;
const double gTarNearRewardWeight = 0.3;
const double gTarHitRewardWeight = 0.4;

double cSceneStrikeAMP::CalcReward(int agent_id) const
{
	double reward = 0;
	if (mMode == eModeTest)
	{
		reward = CalcRewardTest(agent_id);
	}
	else
	{
		reward = CalcRewardTrain(agent_id);
	}
	return reward;
}

double cSceneStrikeAMP::CalcRewardTrain(int agent_id) const
{
	const double tar_far_w = gTarFarRewardWeight;
	const double tar_near_w = gTarNearRewardWeight;
	const double tar_hit_w = gTarHitRewardWeight;

	double reward = 0.0;

	const cSimCharacter* sim_char = GetAgentChar(agent_id);
	tVector root_pos = sim_char->GetRootPos();
	tVector tar_pos = GetTargetPos();
	double near_dist = GetTarNearDist();

	tVector tar_root_delta = tar_pos - root_pos;
	double tar_root_dist_sq = tar_root_delta[0] * tar_root_delta[0] + tar_root_delta[2] * tar_root_delta[2];

	if (TargetHit())
	{
		reward = tar_far_w + tar_near_w + tar_hit_w;
	}
	else if (tar_root_dist_sq < near_dist * near_dist)
	{
		double tar_near_r = CalcRewardTargetNear(agent_id);
		reward = tar_far_w + tar_near_w * tar_near_r;
	}
	else
	{
		double tar_far_r = CalcRewardTargetFar(agent_id);
		reward = tar_far_w * tar_far_r;
	}

	return reward;
}

double cSceneStrikeAMP::CalcRewardTest(int agent_id) const
{
	double reward = 0.0;

	bool episode_end = IsEpisodeEnd();
	if (episode_end)
	{
		eTerminate term = CheckTerminate(agent_id);
		if (term == eTerminateSucc)
		{
			reward = mTimer.GetMaxTime() - GetTime();
		}
	}

	return reward;
}

double cSceneStrikeAMP::CalcRewardTargetNear(int agent_id) const
{
	double reward = 0.0;

	const cSimCharacter* sim_char = GetAgentChar(agent_id);
	tVector root_pos = sim_char->GetRootPos();
	tVector target_pos = GetTargetPos();
	tVector tar_root_delta = target_pos - root_pos;
	tar_root_delta[1] = 0.0;
	double norm = tar_root_delta.norm();

	tVector tar_root_dir = tVector::Zero();
	if (norm > 1e-5)
	{
		tar_root_dir = tar_root_delta / norm;
	}

	for (int i = 0; i < GetNumStrikeBodies(); ++i)
	{
		int b = mStrikeBodies[i];
		const auto& body_part = sim_char->GetBodyPart(b);
		const tVector pos = body_part->GetPos();
		const tVector vel = body_part->GetLinearVelocity();

		tVector tar_body_delta = target_pos - pos;
		double dist_sq = (tar_body_delta).squaredNorm();
		double curr_dist_reward = std::exp(-mTarRewardScale * dist_sq);

		double tar_body_speed = tar_root_dir.dot(vel);
		double cur_vel_reward = tar_body_speed / mHitTarSpeed;
		cur_vel_reward = cMathUtil::Clamp(cur_vel_reward, 0.0, 1.0);
		cur_vel_reward *= cur_vel_reward;

		double curr_reward = 0.2 * curr_dist_reward + 0.8 * cur_vel_reward;
		reward = std::max(reward, curr_reward);
	}

	return reward;
}

double cSceneStrikeAMP::CalcRewardTargetFar(int agent_id) const
{
	const double pos_reward_w = 0.7;
	const double vel_reward_w = 0.3;
	const bool enable_min_tar_vel = mEnableMinTarVel;
	const double near_dist = GetTarNearDist();

	const double tar_speed = GetTargetSpeed();
	const double pos_err_scale = mPosRewardScale;
	const double vel_err_scale = 4 / (tar_speed * tar_speed);

	double reward = 0;

	const auto& ctrl = GetController(agent_id);
	const cSimCharacter* character = GetAgentChar(agent_id);
	bool fallen = HasFallen(*character);
	if (!fallen)
	{
		const cDeepMimicCharController* trl_ctrl = dynamic_cast<const cDeepMimicCharController*>(ctrl.get());
		double prev_action_time = trl_ctrl->GetPrevActionTime();
		const tVector& prev_action_com = trl_ctrl->GetPrevActionCOM();

		const tVector& tar_pos = GetTargetPos();
		tVector root_pos = character->GetRootPos();
		tVector root_tar_delta = tar_pos - root_pos;
		root_tar_delta[1] = 0.0;

		double root_tar_dist = root_tar_delta.norm();
		double root_dist_err = root_tar_dist - near_dist;
		root_dist_err = std::max(root_dist_err, 0.0);
		double pos_reward = std::exp(-pos_err_scale * root_dist_err * root_dist_err);

		double vel_reward = 0;

		if (root_tar_dist < near_dist)
		{
			vel_reward = 1.0;
		}
		else
		{
			double step_dur = trl_ctrl->GetTime() - prev_action_time;

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

	return reward;
}

cSceneStrikeAMP::cSceneStrikeAMP()
{
	mTargetMin = tVector(-0.5, 1.2, 0.6, 0);
	mTargetMax = tVector(0.5, 1.4, 1.1, 0);
	mTargetRadius = 0.2;
	mTargetHit = false;
	mTargetHitTime = gInvalidHitTime;
	mTargetHitResetTime = 2.0;

	mTarRewardScale = 2.0;
	mHitTarSpeed = 1.5;
	mInitHitProb = 0.0;

	mTarNearDist = 1.4;
	mTarFarProb = 0.4;
}

cSceneStrikeAMP::~cSceneStrikeAMP()
{
}

void cSceneStrikeAMP::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cSceneTargetAMP::ParseArgs(parser);

	parser->ParseVector("target_min", mTargetMin);
	parser->ParseVector("target_max", mTargetMax);
	parser->ParseDouble("target_radius", mTargetRadius);
	parser->ParseDouble("target_hit_reset_time", mTargetHitResetTime);

	parser->ParseDouble("tar_reward_scale", mTarRewardScale);
	parser->ParseDouble("hit_tar_speed", mHitTarSpeed);
	parser->ParseDouble("init_hit_prob", mInitHitProb);
	parser->ParseDouble("tar_far_prob", mTarFarProb);
	parser->ParseDouble("tar_near_dist", mTarNearDist);

	parser->ParseInts("strike_bodies", mStrikeBodies);
	parser->ParseInts("fail_tar_contact_bodies", mFailTarContactBodies);
}

void cSceneStrikeAMP::Clear()
{
	cSceneTargetAMP::Clear();
	mStrikeBodies.clear();
}

void cSceneStrikeAMP::SetTargetPosXZ(const tVector& target_pos)
{
	tVector rand_pos = target_pos;
	rand_pos[1] = cMathUtil::RandDouble(mTargetMin[1], mTargetMax[1]);

	const auto& sim_char = GetCharacter();
	tVector char_pos = sim_char->GetRootPos();
	double ground_h = mGround->SampleHeight(char_pos);
	rand_pos[1] += ground_h;

	SetTargetPos(rand_pos);
}

void cSceneStrikeAMP::SetTargetHit(bool hit)
{
	bool prev_hit = TargetHit();
	if (!prev_hit && hit)
	{
		mTargetHitTime = GetTime();
	}

	mTargetHit = hit;
}

bool cSceneStrikeAMP::TargetHit() const
{
	return mTargetHit;
}

double cSceneStrikeAMP::GetTargetRadius() const
{
	return mTargetRadius;
}

const std::vector<int> cSceneStrikeAMP::GetStrikeBodies() const
{
	return mStrikeBodies;
}

double cSceneStrikeAMP::GetRewardSucc(int agent_id)
{
	return gTarFarRewardWeight + gTarNearRewardWeight + gTarHitRewardWeight;
}

double cSceneStrikeAMP::GetTarNearDist() const
{
	return mTarNearDist;
}

std::string cSceneStrikeAMP::GetName() const
{
	return "Strike AMP";
}

void cSceneStrikeAMP::UpdateTarget(double timestep)
{
	cSceneTargetAMP::UpdateTarget(timestep);

	if (!mTargetHit)
	{
		bool tar_hit = CheckTargetHit();
		SetTargetHit(tar_hit);
	}
}

void cSceneStrikeAMP::ResetTarget()
{
	cSceneTargetAMP::ResetTarget();
	ResetTargetHit();

	bool curr_hit = TargetHit();
	if (curr_hit)
	{
		double curr_time = GetTime();
		mTargetHitTime = mRand.RandDouble(curr_time - mTargetHitResetTime, curr_time);
	}
	else
	{
		mTargetHitTime = gInvalidHitTime;
	}
}

void cSceneStrikeAMP::ResetTargetPos()
{
	if (mRand.FlipCoin(mTarFarProb))
	{
		ResetTargetPosFar();
	}
	else
	{
		ResetTargetPosNear();
	}
}

void cSceneStrikeAMP::ResetTargetPosFar()
{
	const auto& sim_char = GetCharacter();
	tVector root_pos = sim_char->GetRootPos();
	tVector rand_pos = tVector::Zero();

	double theta = cMathUtil::RandDouble(-M_PI, M_PI);
	double h = cMathUtil::RandDouble(mTargetMin[1], mTargetMax[1]);
	double dist = cMathUtil::RandDouble(mTargetMin[2], mMaxTargetDist);
	rand_pos[0] = dist * std::cos(theta);
	rand_pos[1] = h;
	rand_pos[2] = dist * -std::sin(theta);

	rand_pos[0] += root_pos[0];
	rand_pos[2] += root_pos[2];

	double ground_h = mGround->SampleHeight(rand_pos);
	rand_pos[1] += ground_h;

	SetTargetHit(false);
	SetTargetPos(rand_pos);
}

void cSceneStrikeAMP::ResetTargetPosNear()
{
	const auto& sim_char = GetCharacter();
	tVector root_pos = sim_char->GetRootPos();
	tVector rand_pos = tVector::Zero();

	double theta = cMathUtil::RandDouble(mTargetMin[0], mTargetMax[0]);
	double h = cMathUtil::RandDouble(mTargetMin[1], mTargetMax[1]);
	double dist = cMathUtil::RandDouble(mTargetMin[2], mTargetMax[2]);
	rand_pos[0] = dist * std::cos(theta);
	rand_pos[1] = h;
	rand_pos[2] = dist * -std::sin(theta);

	rand_pos[0] += root_pos[0];
	rand_pos[2] += root_pos[2];

	double ground_h = mGround->SampleHeight(rand_pos);
	rand_pos[1] += ground_h;

	SetTargetHit(false);
	SetTargetPos(rand_pos);
}

void cSceneStrikeAMP::ResetTargetHit()
{
	if ((mMode == eModeTrain) && (mInitHitProb > 0.0))
	{
		bool hit = cMathUtil::FlipCoin(mInitHitProb);
		SetTargetHit(hit);
	}
}

bool cSceneStrikeAMP::CheckTargetReset(double timestep) const
{
	return false;
}

double cSceneStrikeAMP::CalcHitPhase() const
{
	double phase = 0.0;
	if (TargetHit())
	{
		double time = GetTime();
		double hit_dur = time - mTargetHitTime;
		phase = hit_dur / mTargetHitResetTime;
		phase = cMathUtil::Clamp(phase, 0.0, 1.0);
	}
	return phase;
}

int cSceneStrikeAMP::GetGoalSize(int agent_id) const
{
	return cKinTree::gPosDim + 1;
}

void cSceneStrikeAMP::BuildGoalNormGroups(int agent_id, Eigen::VectorXi& out_groups) const
{
	int goal_size = GetGoalSize(agent_id);
	out_groups = cCharController::gNormGroupNone * Eigen::VectorXi::Ones(goal_size);
}

void cSceneStrikeAMP::RecordGoal(int agent_id, Eigen::VectorXd& out_goal) const
{
	int goal_size = GetGoalSize(agent_id);
	out_goal = std::numeric_limits<double>::quiet_NaN() * Eigen::VectorXd::Ones(goal_size);

	const cSimCharacter* sim_char = GetAgentChar(agent_id);
	tVector tar_pos = GetTargetPos();
	tVector root_pos = sim_char->GetRootPos();
	double ground_h = mGround->SampleHeight(root_pos);
	tar_pos[1] -= ground_h;

	tMatrix origin_trans = sim_char->BuildOriginTrans();
	tar_pos[3] = 1;
	tar_pos = origin_trans * tar_pos;
	tar_pos[3] = 0;

	double hit_phase = CalcHitPhase();

	out_goal.segment(0, cKinTree::gPosDim) = tar_pos.segment(0, cKinTree::gPosDim);
	out_goal[cKinTree::gPosDim] = hit_phase;
}

int cSceneStrikeAMP::GetNumStrikeBodies() const
{
	return static_cast<int>(mStrikeBodies.size());
}

bool cSceneStrikeAMP::CheckTargetHit() const
{
	assert(mStrikeBodies.size() > 0);
	tVector target_pos = GetTargetPos();

	for (int i = 0; i < GetNumChars(); ++i)
	{
		const auto& sim_char = GetCharacter(i);

		tVector root_pos = sim_char->GetRootPos();
		tVector tar_root_delta = target_pos - root_pos;
		tar_root_delta[1] = 0.0;
		double norm = tar_root_delta.norm();

		tVector tar_root_dir = tVector::Zero();
		if (norm > 1e-5)
		{
			tar_root_dir = tar_root_delta / norm;
		}

		for (int j = 0; j < GetNumStrikeBodies(); ++j)
		{
			int b = mStrikeBodies[j];
			const auto& body_part = sim_char->GetBodyPart(b);
			const tVector pos = body_part->GetPos();
			double dist_sq = (target_pos - pos).squaredNorm();

			if (dist_sq < mTargetRadius * mTargetRadius)
			{
				const tVector vel = body_part->GetLinearVelocity();
				double speed = tar_root_dir.dot(vel);

				double tar_speed = mHitTarSpeed;
				if ((speed >= tar_speed) || (tar_speed == 0.0))
				{
					return true;
				}
			}
		}
	}
	return false;
}

bool cSceneStrikeAMP::CheckResetTargetDist() const
{
	return false;
}

bool cSceneStrikeAMP::CheckTarContactFail(int char_id) const
{
	const auto& sim_char = GetCharacter(char_id);
	const tVector& tar_pos = GetTargetPos();

	int num_bodies = static_cast<int>(mFailTarContactBodies.size());
	for (int i = 0; i < num_bodies; ++i)
	{
		int body_id = mFailTarContactBodies[i];
		const auto& sim_body = sim_char->GetBodyPart(body_id);
		tVector body_pos = sim_body->GetPos();

		double dist_sq = (tar_pos - body_pos).squaredNorm();
		if (dist_sq < mTargetRadius * mTargetRadius)
		{
			return true;
		}
	}
	return false;
}

bool cSceneStrikeAMP::CheckTarHitSucc() const
{
	bool succ = false;

	bool tar_hit = TargetHit();
	if (tar_hit)
	{
		double time = GetTime();
		double hit_dur = time - mTargetHitTime;
		if (hit_dur >= mTargetHitResetTime)
		{
			succ = true;
		}
	}

	return succ;
}

cSceneStrikeAMP::eTerminate cSceneStrikeAMP::CheckTerminateTarget(int agent_id) const
{
	eTerminate terminated = cSceneTargetAMP::CheckTerminateTarget(agent_id);

	if (terminated == eTerminate::eTerminateNull)
	{
		const auto& agent_char = GetAgentChar(agent_id);
		bool contact_fail = CheckTarContactFail(agent_char->GetID());
		terminated = (contact_fail) ? eTerminateFail : terminated;
	}

	if (terminated == eTerminateNull)
	{
		bool term_succ = CheckTarHitSucc();
		terminated = (term_succ) ? eTerminateSucc : terminated;
	}

	return terminated;
}
