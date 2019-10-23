#include "SceneImitate.h"
#include "sim/RBDUtil.h"
#include "sim/CtController.h"
#include "util/FileUtil.h"
#include "util/JsonUtil.h"

double cSceneImitate::CalcRewardImitate(const cSimCharacter& sim_char, const cKinCharacter& kin_char) const
{
	double pose_w = 0.5;
	double vel_w = 0.05;
	double end_eff_w = 0.15;
	double root_w = 0.2;
	double com_w = 0.1;

	double total_w = pose_w + vel_w + end_eff_w + root_w + com_w;
	pose_w /= total_w;
	vel_w /= total_w;
	end_eff_w /= total_w;
	root_w /= total_w;
	com_w /= total_w;

	int num_joints = sim_char.GetNumJoints();
	assert(num_joints == mJointWeights.size());
	
	const double pose_scale = 2.0 / 15 * num_joints;
	const double vel_scale = 0.1 / 15 * num_joints;
	const double end_eff_scale = 10;
	const double root_scale = 5;
	const double com_scale = 10;
	const double err_scale = 1;

	const auto& joint_mat = sim_char.GetJointMat();
	const auto& body_defs = sim_char.GetBodyDefs();
	double reward = 0;

	const Eigen::VectorXd& pose0 = sim_char.GetPose();
	const Eigen::VectorXd& vel0 = sim_char.GetVel();
	const Eigen::VectorXd& pose1 = kin_char.GetPose();
	const Eigen::VectorXd& vel1 = kin_char.GetVel();
	tMatrix origin_trans = sim_char.BuildOriginTrans();
	tMatrix kin_origin_trans = kin_char.BuildOriginTrans();

	tVector com0_world = sim_char.CalcCOM();
	tVector com_vel0_world = sim_char.CalcCOMVel();
	tVector com1_world;
	tVector com_vel1_world;
	cRBDUtil::CalcCoM(joint_mat, body_defs, pose1, vel1, com1_world, com_vel1_world);

	int root_id = sim_char.GetRootID();
	tVector root_pos0 = cKinTree::GetRootPos(joint_mat, pose0);
	tVector root_pos1 = cKinTree::GetRootPos(joint_mat, pose1);
	tQuaternion root_rot0 = cKinTree::GetRootRot(joint_mat, pose0);
	tQuaternion root_rot1 = cKinTree::GetRootRot(joint_mat, pose1);
	tVector root_vel0 = cKinTree::GetRootVel(joint_mat, vel0);
	tVector root_vel1 = cKinTree::GetRootVel(joint_mat, vel1);
	tVector root_ang_vel0 = cKinTree::GetRootAngVel(joint_mat, vel0);
	tVector root_ang_vel1 = cKinTree::GetRootAngVel(joint_mat, vel1);

	double pose_err = 0;
	double vel_err = 0;
	double end_eff_err = 0;
	double root_err = 0;
	double com_err = 0;
	double heading_err = 0;

	double root_rot_w = mJointWeights[root_id];
	pose_err += root_rot_w * cKinTree::CalcRootRotErr(joint_mat, pose0, pose1);
	vel_err += root_rot_w * cKinTree::CalcRootAngVelErr(joint_mat, vel0, vel1);

	for (int j = root_id + 1; j < num_joints; ++j)
	{
		double w = mJointWeights[j];
		double curr_pose_err = cKinTree::CalcPoseErr(joint_mat, j, pose0, pose1);
		double curr_vel_err = cKinTree::CalcVelErr(joint_mat, j, vel0, vel1);
		pose_err += w * curr_pose_err;
		vel_err += w * curr_vel_err;

		bool is_end_eff = sim_char.IsEndEffector(j);
		if (is_end_eff)
		{
			tVector pos0 = sim_char.CalcJointPos(j);
			tVector pos1 = cKinTree::CalcJointWorldPos(joint_mat, pose1, j);
			double ground_h0 = mGround->SampleHeight(pos0);
			double ground_h1 = kin_char.GetOriginPos()[1];

			tVector pos_rel0 = pos0 - root_pos0;
			tVector pos_rel1 = pos1 - root_pos1;
			pos_rel0[1] = pos0[1] - ground_h0;
			pos_rel1[1] = pos1[1] - ground_h1;

			pos_rel0 = origin_trans * pos_rel0;
			pos_rel1 = kin_origin_trans * pos_rel1;

			double curr_end_err = (pos_rel1 - pos_rel0).squaredNorm();
			end_eff_err += curr_end_err;
		}
	}

	double root_ground_h0 = mGround->SampleHeight(sim_char.GetRootPos());
	double root_ground_h1 = kin_char.GetOriginPos()[1];
	root_pos0[1] -= root_ground_h0;
	root_pos1[1] -= root_ground_h1;
	double root_pos_err = (root_pos0 - root_pos1).squaredNorm();
	
	double root_rot_err = cMathUtil::QuatDiffTheta(root_rot0, root_rot1);
	root_rot_err *= root_rot_err;

	double root_vel_err = (root_vel1 - root_vel0).squaredNorm();
	double root_ang_vel_err = (root_ang_vel1 - root_ang_vel0).squaredNorm();

	root_err = root_pos_err
			+ 0.1 * root_rot_err
			+ 0.01 * root_vel_err
			+ 0.001 * root_ang_vel_err;
	com_err = 0.1 * (com_vel1_world - com_vel0_world).squaredNorm();

	double pose_reward = exp(-err_scale * pose_scale * pose_err);
	double vel_reward = exp(-err_scale * vel_scale * vel_err);
	double end_eff_reward = exp(-err_scale * end_eff_scale * end_eff_err);
	double root_reward = exp(-err_scale * root_scale * root_err);
	double com_reward = exp(-err_scale * com_scale * com_err);

	reward = pose_w * pose_reward + vel_w * vel_reward + end_eff_w * end_eff_reward
		+ root_w * root_reward + com_w * com_reward;

	return reward;
}

cSceneImitate::cSceneImitate()
{
	mEnableRandRotReset = false;
	mSyncCharRootPos = true;
	mSyncCharRootRot = false;
	mMotionFile = "";
	mEnableRootRotFail = false;
	mHoldEndFrame = 0;
}

cSceneImitate::~cSceneImitate()
{
}

void cSceneImitate::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cRLSceneSimChar::ParseArgs(parser);
	parser->ParseString("motion_file", mMotionFile);
	parser->ParseBool("enable_rand_rot_reset", mEnableRandRotReset);
	parser->ParseBool("sync_char_root_pos", mSyncCharRootPos);
	parser->ParseBool("sync_char_root_rot", mSyncCharRootRot);
	parser->ParseBool("enable_root_rot_fail", mEnableRootRotFail);
	parser->ParseDouble("hold_end_frame", mHoldEndFrame);
}

void cSceneImitate::Init()
{
	mKinChar.reset();
	BuildKinChar();

	cRLSceneSimChar::Init();
	InitJointWeights();
}

double cSceneImitate::CalcReward(int agent_id) const
{
	const cSimCharacter* sim_char = GetAgentChar(agent_id);
	bool fallen = HasFallen(*sim_char);

	double r = 0;
	int max_id = 0;
	if (!fallen)
	{
		r = CalcRewardImitate(*sim_char, *mKinChar);
	}
	return r;
}

const std::shared_ptr<cKinCharacter>& cSceneImitate::GetKinChar() const
{
	return mKinChar;
}

void cSceneImitate::EnableRandRotReset(bool enable)
{
	mEnableRandRotReset = enable;
}

bool cSceneImitate::EnabledRandRotReset() const
{
	bool enable = mEnableRandRotReset;
	return enable;
}

cSceneImitate::eTerminate cSceneImitate::CheckTerminate(int agent_id) const
{
	eTerminate terminated = cRLSceneSimChar::CheckTerminate(agent_id);
	if (terminated == eTerminateNull)
	{
		bool end_motion = false;
		const auto& kin_char = GetKinChar();
		const cMotion& motion = kin_char->GetMotion();

		if (motion.GetLoop() == cMotion::eLoopNone)
		{
			double dur = motion.GetDuration();
			double kin_time = kin_char->GetTime();
			end_motion = kin_time > dur + mHoldEndFrame;
		}
		else
		{
			end_motion = kin_char->IsMotionOver();
		}

		terminated = (end_motion) ? eTerminateFail : terminated;
	}
	return terminated;
}

std::string cSceneImitate::GetName() const
{
	return "Imitate";
}

bool cSceneImitate::BuildCharacters()
{
	bool succ = cRLSceneSimChar::BuildCharacters();
	if (EnableSyncChar())
	{
		SyncCharacters();
	}
	return succ;
}

void cSceneImitate::CalcJointWeights(const std::shared_ptr<cSimCharacter>& character, Eigen::VectorXd& out_weights) const
{
	int num_joints = character->GetNumJoints();
	out_weights = Eigen::VectorXd::Ones(num_joints);
	for (int j = 0; j < num_joints; ++j)
	{
		double curr_w = character->GetJointDiffWeight(j);
		out_weights[j] = curr_w;
	}

	double sum = out_weights.lpNorm<1>();
	out_weights /= sum;
}

bool cSceneImitate::BuildController(const cCtrlBuilder::tCtrlParams& ctrl_params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = cSceneSimChar::BuildController(ctrl_params, out_ctrl);
	if (succ)
	{
		auto ct_ctrl = dynamic_cast<cCtController*>(out_ctrl.get());
		if (ct_ctrl != nullptr)
		{
			const auto& kin_char = GetKinChar();
			double cycle_dur = kin_char->GetMotionDuration();
			ct_ctrl->SetCyclePeriod(cycle_dur);
		}
	}
	return succ;
}

void cSceneImitate::BuildKinChar()
{
	bool succ = BuildKinCharacter(0, mKinChar);
	if (!succ)
	{
		printf("Failed to build kin character\n");
		assert(false);
	}
}

bool cSceneImitate::BuildKinCharacter(int id, std::shared_ptr<cKinCharacter>& out_char) const
{
	auto kin_char = std::shared_ptr<cKinCharacter>(new cKinCharacter());
	const cSimCharacter::tParams& sim_char_params = mCharParams[0];
	cKinCharacter::tParams kin_char_params;

	kin_char_params.mID = id;
	kin_char_params.mCharFile = sim_char_params.mCharFile;
	kin_char_params.mOrigin = sim_char_params.mInitPos;
	kin_char_params.mLoadDrawShapes = false;
	kin_char_params.mMotionFile = mMotionFile;

	bool succ = kin_char->Init(kin_char_params);
	if (succ)
	{
		out_char = kin_char;
	}
	return succ;
}

void cSceneImitate::UpdateCharacters(double timestep)
{
	UpdateKinChar(timestep);
	cRLSceneSimChar::UpdateCharacters(timestep);
}

void cSceneImitate::UpdateKinChar(double timestep)
{
	const auto& kin_char = GetKinChar();
	double prev_phase = kin_char->GetPhase();
	kin_char->Update(timestep);
	double curr_phase = kin_char->GetPhase();

	if (curr_phase < prev_phase)
	{
		const auto& sim_char = GetCharacter();
		SyncKinCharNewCycle(*sim_char, *kin_char);
	}
}

void cSceneImitate::ResetCharacters()
{
	cRLSceneSimChar::ResetCharacters();

	ResetKinChar();
	if (EnableSyncChar())
	{
		SyncCharacters();
	}
}

void cSceneImitate::ResetKinChar()
{
	double rand_time = CalcRandKinResetTime();

	const cSimCharacter::tParams& char_params = mCharParams[0];
	const auto& kin_char = GetKinChar();

	kin_char->Reset();
	kin_char->SetOriginRot(tQuaternion::Identity());
	kin_char->SetOriginPos(char_params.mInitPos); // reset origin
	kin_char->SetTime(rand_time);
	kin_char->Pose(rand_time);

	if (EnabledRandRotReset())
	{
		double rand_theta = mRand.RandDouble(-M_PI, M_PI);
		kin_char->RotateOrigin(cMathUtil::EulerToQuaternion(tVector(0, rand_theta, 0, 0)));
	}
}

void cSceneImitate::SyncCharacters()
{
	const auto& kin_char = GetKinChar();
	const Eigen::VectorXd& pose = kin_char->GetPose();
	const Eigen::VectorXd& vel = kin_char->GetVel();
	
	const auto& sim_char = GetCharacter();
	sim_char->SetPose(pose);
	sim_char->SetVel(vel);

	const auto& ctrl = sim_char->GetController();
	auto ct_ctrl = dynamic_cast<cCtController*>(ctrl.get());
	if (ct_ctrl != nullptr)
	{
		double kin_time = GetKinTime();
		ct_ctrl->SetInitTime(kin_time);
	}
}

bool cSceneImitate::EnableSyncChar() const
{
	const auto& kin_char = GetKinChar();
	return kin_char->HasMotion();
}

void cSceneImitate::InitCharacterPosFixed(const std::shared_ptr<cSimCharacter>& out_char)
{
	// nothing to see here
}

void cSceneImitate::InitJointWeights()
{
	CalcJointWeights(GetCharacter(), mJointWeights);
}

void cSceneImitate::ResolveCharGroundIntersect()
{
	cRLSceneSimChar::ResolveCharGroundIntersect();

	if (EnableSyncChar())
	{
		SyncKinCharRoot();
	}
}

void cSceneImitate::ResolveCharGroundIntersect(const std::shared_ptr<cSimCharacter>& out_char) const
{
	cRLSceneSimChar::ResolveCharGroundIntersect(out_char);
}

void cSceneImitate::SyncKinCharRoot()
{
	const auto& sim_char = GetCharacter();
	tVector sim_root_pos = sim_char->GetRootPos();
	double sim_heading = sim_char->CalcHeading();

	const auto& kin_char = GetKinChar();
	double kin_heading = kin_char->CalcHeading();

	tQuaternion drot = tQuaternion::Identity();
	if (mSyncCharRootRot)
	{
		drot = cMathUtil::AxisAngleToQuaternion(tVector(0, 1, 0, 0), sim_heading - kin_heading);
	}

	kin_char->RotateRoot(drot);
	kin_char->SetRootPos(sim_root_pos);
}

void cSceneImitate::SyncKinCharNewCycle(const cSimCharacter& sim_char, cKinCharacter& out_kin_char) const
{
	if (mSyncCharRootRot)
	{
		double sim_heading = sim_char.CalcHeading();
		double kin_heading = out_kin_char.CalcHeading();
		tQuaternion drot = cMathUtil::AxisAngleToQuaternion(tVector(0, 1, 0, 0), sim_heading - kin_heading);
		out_kin_char.RotateRoot(drot);
	}

	if (mSyncCharRootPos)
	{
		tVector sim_root_pos = sim_char.GetRootPos();
		tVector kin_root_pos = out_kin_char.GetRootPos();
		kin_root_pos[0] = sim_root_pos[0];
		kin_root_pos[2] = sim_root_pos[2];

		tVector origin = out_kin_char.GetOriginPos();
		double dh = kin_root_pos[1] - origin[1];
		double ground_h = mGround->SampleHeight(kin_root_pos);
		kin_root_pos[1] = ground_h + dh;

		out_kin_char.SetRootPos(kin_root_pos);
	}
}

double cSceneImitate::GetKinTime() const
{
	const auto& kin_char = GetKinChar();
	return kin_char->GetTime();
}

bool cSceneImitate::CheckKinNewCycle(double timestep) const
{
	bool new_cycle = false;
	const auto& kin_char = GetKinChar();
	if (kin_char->GetMotion().EnableLoop())
	{
		double cycle_dur = kin_char->GetMotionDuration();
		double time = GetKinTime();
		new_cycle = cMathUtil::CheckNextInterval(timestep, time, cycle_dur);
	}
	return new_cycle;
}


bool cSceneImitate::HasFallen(const cSimCharacter& sim_char) const
{
	bool fallen = cRLSceneSimChar::HasFallen(sim_char);
	if (mEnableRootRotFail)
	{
		fallen |= CheckRootRotFail(sim_char);
	}

	return fallen;
}

bool cSceneImitate::CheckRootRotFail(const cSimCharacter& sim_char) const
{
	const auto& kin_char = GetKinChar();
	bool fail = CheckRootRotFail(sim_char, *kin_char);
	return fail;
}

bool cSceneImitate::CheckRootRotFail(const cSimCharacter& sim_char, const cKinCharacter& kin_char) const
{
	const double threshold = 0.5 * M_PI;

	tQuaternion sim_rot = sim_char.GetRootRotation();
	tQuaternion kin_rot = kin_char.GetRootRotation();
	double rot_diff = cMathUtil::QuatDiffTheta(sim_rot, kin_rot);
	return rot_diff > threshold;
}

double cSceneImitate::CalcRandKinResetTime()
{
	const auto& kin_char = GetKinChar();
	double dur = kin_char->GetMotionDuration();
	double rand_time = cMathUtil::RandDouble(0, dur);
	return rand_time;
}