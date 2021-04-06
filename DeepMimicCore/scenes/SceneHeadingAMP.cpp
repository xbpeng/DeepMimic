#include "SceneHeadingAMP.h"

double cSceneHeadingAMP::CalcReward(int agent_id) const
{
	const bool enable_min_tar_vel = mEnableMinTarVel;

	double reward = 0;

	const auto& ctrl = GetController(agent_id);
	const cSimCharacter* character = GetAgentChar(agent_id);
	bool fallen = HasFallen(*character);
	if (!fallen)
	{
		const cDeepMimicCharController* char_ctrl = dynamic_cast<const cDeepMimicCharController*>(ctrl.get());
		double prev_action_time = char_ctrl->GetPrevActionTime();
		const tVector& prev_action_com = char_ctrl->GetPrevActionCOM();
		tVector com = character->CalcCOM();

		double tar_speed = mTargetSpeed;
		tVector tar_dir = tVector(std::cos(mTargetHeading), 0, -std::sin(mTargetHeading), 0);

		double step_dur = char_ctrl->GetTime() - prev_action_time;
		tVector avg_vel = (com - prev_action_com) / step_dur;
		avg_vel[1] = 0.0;
		double avg_speed = tar_dir.dot(avg_vel);

		double vel_reward = 0;
		if (avg_speed > 0.0)
		{
			double vel_err = tar_speed - avg_speed;
			if (enable_min_tar_vel)
			{
				vel_err = std::max(vel_err, 0.0);
			}
			vel_reward = std::exp(-mVelRewardScale * vel_err * vel_err);
		}

		reward = vel_reward;
	}

	return reward;
}

cSceneHeadingAMP::cSceneHeadingAMP()
{
	mTargetTimerParams.mTimeMin = 0.2;
	mTargetTimerParams.mTimeMax = 0.5;

	mMaxHeadingTurnRate = 0.15;
	mSharpTurnProb = 0.025;
	mSpeedChangeProb = 0.1;
	
	mTargetSpeedMin = mTargetSpeed;
	mTargetSpeedMax = mTargetSpeed;
	mTargetHeading = 0;
	mVelRewardScale = 1.0;

	EnableTargetPos(false);
	EnableRandSpeed(true);
}

cSceneHeadingAMP::~cSceneHeadingAMP()
{
}

void cSceneHeadingAMP::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cSceneTargetAMP::ParseArgs(parser);

	parser->ParseDouble("max_heading_turn_rate", mMaxHeadingTurnRate);
	parser->ParseDouble("sharp_turn_prob", mSharpTurnProb);
	parser->ParseDouble("speed_change_prob", mSpeedChangeProb);

	mTargetSpeedMin = mTargetSpeed;
	mTargetSpeedMax = mTargetSpeed;
	parser->ParseDouble("tar_speed_min", mTargetSpeedMin);
	parser->ParseDouble("tar_speed_max", mTargetSpeedMax);
	mTargetSpeed = cMathUtil::Clamp(mTargetSpeed, mTargetSpeedMin, mTargetSpeedMax);

	parser->ParseDouble("vel_reward_scale", mVelRewardScale);
}

void cSceneHeadingAMP::SetTargetSpeed(double speed)
{
	speed = cMathUtil::Clamp(speed, mTargetSpeedMin, mTargetSpeedMax);
	cSceneTargetAMP::SetTargetSpeed(speed);
}

double cSceneHeadingAMP::GetTargetHeading() const
{
	return mTargetHeading;
}

void cSceneHeadingAMP::SetTargetHeading(double heading)
{
	mTargetHeading = heading;
}

tVector cSceneHeadingAMP::GetTargetPos() const
{
	return mTargetPos;
}

void cSceneHeadingAMP::SetTargetPos(const tVector& target_pos)
{
	mTargetPos = target_pos;
}

bool cSceneHeadingAMP::EnableTargetPos() const
{
	return mEnableTargetPos;
}

void cSceneHeadingAMP::EnableTargetPos(bool enable)
{
	mEnableTargetPos = enable;
}

bool cSceneHeadingAMP::EnableRandSpeed() const
{
	return mEnableRandSpeed;
}

void cSceneHeadingAMP::EnableRandSpeed(bool enable)
{
	mEnableRandSpeed = enable;
}

int cSceneHeadingAMP::GetGoalSize(int agent_id) const
{
	return 3;
}

void cSceneHeadingAMP::RecordGoal(int agent_id, Eigen::VectorXd& out_goal) const
{
	int g_size = GetGoalSize(agent_id);
	out_goal.resize(g_size);

	double tar_heading = GetTargetHeading();
	double tar_speed = GetTargetSpeed();

	const auto& sim_char = GetAgentChar(agent_id);
	double char_heading = sim_char->CalcHeading();
	tar_heading -= char_heading;

	out_goal[0] = std::cos(tar_heading);
	out_goal[1] = -std::sin(tar_heading);
	out_goal[2] = tar_speed;
}

std::string cSceneHeadingAMP::GetName() const
{
	return "Heading AMP";
}

void cSceneHeadingAMP::UpdateTargetHeading()
{
	double new_heading = GetTargetHeading();

	if (EnableTargetPos())
	{
		const auto& sim_char = GetCharacter();
		tVector com = sim_char->CalcCOM();
		tVector delta = GetTargetPos() - com;
		if (delta.squaredNorm() > 0.01)
		{
			new_heading = std::atan2(-delta[2], delta[0]);
		}
	}
	else
	{
		bool sharp_turn = mRand.FlipCoin(mSharpTurnProb);
		double delta_heading = 0;
		if (sharp_turn)
		{
			delta_heading = mRand.RandDouble(-M_PI, M_PI);
		}
		else
		{
			delta_heading = mRand.RandDoubleNorm(0, mMaxHeadingTurnRate);
		}
		new_heading += delta_heading;
	}

	SetTargetHeading(new_heading);
}

void cSceneHeadingAMP::UpdateTargetSpeed()
{
	bool change_speed = mRand.FlipCoin(mSpeedChangeProb);
	if (change_speed)
	{
		double new_speed = mRand.RandDouble(mTargetSpeedMin, mTargetSpeedMax);
		SetTargetSpeed(new_speed);
	}
}

void cSceneHeadingAMP::UpdateTarget(double timestep)
{
	cSceneTargetAMP::UpdateTarget(timestep);
	
	if (mTargetTimer.IsEnd() || EnableTargetPos())
	{
		UpdateTargetHeading();
	}

	if (mTargetTimer.IsEnd() && EnableRandSpeed())
	{
		UpdateTargetSpeed();
	}
}

void cSceneHeadingAMP::ResetTarget()
{
	cSceneTargetAMP::ResetTarget();

	double heading = 0;
	double speed = mRand.RandDouble(mTargetSpeedMin, mTargetSpeedMax);

	SetTargetHeading(heading);
	SetTargetSpeed(speed);
}

bool cSceneHeadingAMP::CheckTarDistFail(int char_id) const
{
	return false;
}