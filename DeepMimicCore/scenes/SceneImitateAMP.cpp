#include "SceneImitateAMP.h"
#include "anim/ClipsController.h"
#include "sim/CtController.h"

double cSceneImitateAMP::TimeWarpCost(const Eigen::VectorXd* data0, const Eigen::VectorXd* data1)
{
	assert(data0->size() == data1->size());
	assert(data0->size() % 3 == 0);

	double cost = 0.0;

	int num_points = data0->size() / 3;
	for (int p = 0; p < num_points; ++p)
	{
		int idx = p * cKinTree::gPosDim;
		tVector pt0 = tVector((*data0)(idx), (*data0)(idx + 1), (*data0)(idx + 2), 0);
		tVector pt1 = tVector((*data1)(idx), (*data1)(idx + 1), (*data1)(idx + 2), 0);
		double dist = (pt1 - pt0).norm();
		cost += dist;
	}
	
	cost /= num_points;

	return cost;
}

cSceneImitateAMP::cSceneImitateAMP()
{
	mEnableAMPObsLocalRoot = false;
	mTestTimeWarp = true;
	mTimeWarper = nullptr;
}

cSceneImitateAMP::~cSceneImitateAMP()
{
}

void cSceneImitateAMP::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cSceneImitate::ParseArgs(parser);

	parser->ParseBool("enable_amp_obs_local_root", mEnableAMPObsLocalRoot);
	parser->ParseBool("enable_test_time_warp", mTestTimeWarp);
}

void cSceneImitateAMP::Init()
{
	cSceneImitate::Init();

	InitHist();

	if (mTestTimeWarp)
	{
		BuildTimeWarper();
	}
}

void cSceneImitateAMP::Reset()
{
	cSceneImitate::Reset();

	InitHist();

	if (EnableTestTimeWarp())
	{
		ResetTimeWarper();
	}
}

bool cSceneImitateAMP::EnableAMPTaskReward() const
{
	return false;
}

int cSceneImitateAMP::GetAMPObsSize() const
{
	const int num_timesteps = 2;

	int pose_size = GetAMPObsPoseSize();
	int vel_size = GetAMPObsVelSize();
	int obs_size = (pose_size + vel_size) * num_timesteps;

	return obs_size;
}

void cSceneImitateAMP::GetAMPObsOffset(Eigen::VectorXd& out_data) const
{
	out_data = Eigen::VectorXd::Zero(GetAMPObsSize());
}

void cSceneImitateAMP::GetAMPObsScale(Eigen::VectorXd& out_data) const
{
	out_data = Eigen::VectorXd::Ones(GetAMPObsSize());
}

void cSceneImitateAMP::GetAMPObsNormGroup(Eigen::VectorXi& out_data) const
{
	out_data = cCharController::gNormGroupSingle * Eigen::VectorXi::Ones(GetAMPObsSize());
}

void cSceneImitateAMP::RecordAMPObsAgent(int agent_id, Eigen::VectorXd& out_data)
{
	const auto* character = GetAgentChar(agent_id);
	const auto& pose = character->GetPose();
	const auto& vel = character->GetVel();

	tVector root_pos = cKinTree::GetRootPos(pose);
	const auto& ground = GetGround();
	double ground_h = ground->SampleHeight(root_pos);

	BuildAMPObs(agent_id, mPrevPose, mPrevVel, pose, vel, ground_h, out_data);
}

void cSceneImitateAMP::RecordAMPObsExpert(int agent_id, Eigen::VectorXd& out_data)
{
	const auto& kin_char = GetKinChar();
	const cMotion* motion = SampleExpertMotion(kin_char.get());
	double motion_duration = motion->GetDuration();
	double rand_kin_time = mRand.RandDouble(0, motion_duration);

	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	motion->CalcFrame(rand_kin_time, pose);
	motion->CalcFrameVel(rand_kin_time, vel);

	const auto& ctrl = GetController();
	const cCtController* ct_ctrl = dynamic_cast<const cCtController*>(ctrl.get());
	double dt = 1.0 / ct_ctrl->GetUpdateRate();
	double prev_kin_time = rand_kin_time - dt;

	Eigen::VectorXd prev_pose;
	Eigen::VectorXd prev_vel;
	motion->CalcFrame(prev_kin_time, prev_pose);
	motion->CalcFrameVel(prev_kin_time, prev_vel);

	double ground_h = kin_char->GetOriginPos()[1];

	BuildAMPObs(agent_id, prev_pose, prev_vel, pose, vel, ground_h, out_data);
}

void cSceneImitateAMP::NewActionUpdate(int agent_id)
{
	cSceneImitate::NewActionUpdate(agent_id);

	UpdateHist();

	if (EnableTestTimeWarp())
	{
		UpdateTimeWarper();
	}
}

void cSceneImitateAMP::InitHist()
{
	const auto& ctrl = GetController();
	const cCtController* ct_ctrl = dynamic_cast<const cCtController*>(ctrl.get());
	double curr_time = ct_ctrl->GetTime();
	double dt = 1.0 / ct_ctrl->GetUpdateRate();

	const auto& kin_char = GetKinChar();
	double prev_time = curr_time - dt;

	kin_char->CalcPose(prev_time, mPrevPose);
	kin_char->CalcVel(prev_time, mPrevVel);
}

void cSceneImitateAMP::UpdateHist()
{
	const auto& sim_char = GetCharacter();
	mPrevPose = sim_char->GetPose();
	mPrevVel = sim_char->GetVel();
}

double cSceneImitateAMP::CalcReward(int agent_id) const
{
	double reward = 0.0;
	if (EnableTestTimeWarp() && (mMode == eModeTest))
	{
		reward = CalcRewardTimeWarp(agent_id);
	}
	return reward;
}

cSceneImitateAMP::eTerminate cSceneImitateAMP::CheckTerminate(int agent_id) const
{
	eTerminate terminated = cRLSceneSimChar::CheckTerminate(agent_id);
	return terminated;
}

double cSceneImitateAMP::CalcRewardTimeWarp(int agent_id) const
{
	const double term_step_cost = 1.0;

	double reward = 0.0;
	if (IsEpisodeEnd())
	{
		double cost = mTimeWarper->CalcAlignment();

		int timesteps = mTimeWarper->GetSampleCount0();
		int buffer_size = mTimeWarper->GetBufferSize();
		int remainder_steps = buffer_size - timesteps;
		cost += remainder_steps * term_step_cost;

		reward = cost;
	}
	return reward;
}

std::string cSceneImitateAMP::GetName() const
{
	return "Imitate AMP";
}

int cSceneImitateAMP::GetAMPObsPoseSize() const
{
	int pos_dim = cKinTree::gPosDim;
	const auto& character = GetCharacter();

	int size = 0;

	size += 1; // +1 for root y
	size += 2 * cKinTree::gPosDim; // root rot
	size += character->GetNumEndEffectors() * pos_dim;

	int num_joints = character->GetNumJoints();
	for (int j = 1; j < num_joints; ++j)
	{
		int joint_param_size = 0.0;
		cKinTree::eJointType joint_type = character->GetJointType(j);

		if (joint_type == cKinTree::eJointTypeSpherical)
		{
			joint_param_size = 2 * cKinTree::gPosDim;
		}
		else
		{
			joint_param_size = cKinTree::GetJointParamSize(joint_type);
		}

		size += joint_param_size;
	}
	return size;
}

int cSceneImitateAMP::GetAMPObsVelSize() const
{
	const int vel_dim = cKinTree::gVelDim;
	const int ang_vel_dim = cKinTree::gAngVelDim;
	const auto& character = GetCharacter();

	int root_id = character->GetRootID();
	int root_param_size = character->GetParamSize(root_id);

	int num_dofs = character->GetNumDof();
	int vel_size = num_dofs - root_param_size + vel_dim + ang_vel_dim;
	
	return vel_size;
}

const cMotion* cSceneImitateAMP::SampleExpertMotion(const cKinCharacter* kin_char) const
{
	const cMotion* motion = nullptr;
	const auto kin_ctrl = kin_char->GetController();
	const cClipsController* clips_ctrl = dynamic_cast<const cClipsController*>(kin_ctrl.get());

	if (clips_ctrl != nullptr)
	{
		int rand_motion_id = clips_ctrl->SampleMotionID();
		motion = &(clips_ctrl->GetMotion(rand_motion_id));
	}
	else
	{
		motion = kin_char->GetMotion();
	}

	return motion;
}

void cSceneImitateAMP::BuildAMPObs(int agent_id, const Eigen::VectorXd& prev_pose, const Eigen::VectorXd& prev_vel,
								const Eigen::VectorXd& pose, const Eigen::VectorXd& vel,
								double ground_h, Eigen::VectorXd& out_data) const
{
	// fill with nans to make sure we don't forget to set anything
	out_data = std::numeric_limits<double>::quiet_NaN() * Eigen::VectorXd::Ones(GetAMPObsSize());

	tQuaternion ref_origin_rot = cKinTree::CalcHeadingRot(pose);

	int param_offset = 0;
	param_offset += RecordAMPObsPose(agent_id, pose, ground_h, ref_origin_rot, param_offset, out_data);
	param_offset += RecordAMPObsPose(agent_id, prev_pose, ground_h, ref_origin_rot, param_offset, out_data);

	param_offset += RecordAMPObsVel(agent_id, vel, ref_origin_rot, param_offset, out_data);
	param_offset += RecordAMPObsVel(agent_id, prev_vel, ref_origin_rot, param_offset, out_data);

	assert(param_offset == out_data.size());
}

int cSceneImitateAMP::RecordAMPObsPose(int agent_id, const Eigen::VectorXd& pose, double ground_h, 
										const tQuaternion& ref_origin_rot, int param_offset, 
										Eigen::VectorXd& out_data) const
{
	const int pos_dim = cKinTree::gPosDim;

	const auto* character = GetAgentChar(agent_id);
	const auto& joint_mat = character->GetJointMat();
	const auto& body_defs = character->GetBodyDefs();
	const tVector root_pos = cKinTree::GetRootPos(pose);
	tQuaternion root_rot = cKinTree::GetRootRot(pose);

	int curr_param_offset = param_offset;

	double root_h = root_pos[1] - ground_h;
	out_data[curr_param_offset] = root_h;
	curr_param_offset += 1;

	if (mEnableAMPObsLocalRoot)
	{
		root_rot = ref_origin_rot * root_rot;
	}

	tVector root_rot_norm;
	tVector root_rot_tan;
	cMathUtil::CalcNormalTangent(root_rot, root_rot_norm, root_rot_tan);
	out_data.segment(curr_param_offset, pos_dim) = root_rot_norm.segment(0, pos_dim);
	out_data.segment(curr_param_offset + pos_dim, pos_dim) = root_rot_tan.segment(0, pos_dim);
	curr_param_offset += 2 * pos_dim;
	
	int num_joints = character->GetNumJoints();
	for (int j = 1; j < num_joints; ++j)
	{
		cKinTree::eJointType joint_type = character->GetJointType(j);
		int joint_param_offset = character->GetParamOffset(j);
		int joint_param_size = character->GetParamSize(j);

		auto joint_param = pose.segment(joint_param_offset, joint_param_size);
		if (joint_type == cKinTree::eJointTypeSpherical)
		{
			joint_param_size = 2 * cKinTree::gPosDim;
			tQuaternion joint_rot = cMathUtil::VecToQuat(joint_param);
			tVector joint_rot_norm;
			tVector joint_rot_tan;
			cMathUtil::CalcNormalTangent(joint_rot, joint_rot_norm, joint_rot_tan);
			out_data.segment(curr_param_offset, cKinTree::gPosDim) = joint_rot_norm.segment(0, cKinTree::gPosDim);
			out_data.segment(curr_param_offset + cKinTree::gPosDim, cKinTree::gPosDim) = joint_rot_tan.segment(0, cKinTree::gPosDim);
		}
		else
		{
			out_data.segment(curr_param_offset, joint_param_size) = joint_param;
		}
		curr_param_offset += joint_param_size;
	}

	int num_end_effs = character->GetNumEndEffectors();
	const auto& end_effs = character->GetEndEffectors();
	for (int e = 0; e < num_end_effs; ++e)
	{
		int joint_id = end_effs[e];
		tVector curr_pos = cKinTree::CalcBodyPartPos(joint_mat, body_defs, pose, joint_id);
		curr_pos -= root_pos;
		curr_pos = cMathUtil::QuatRotVec(ref_origin_rot, curr_pos);
		out_data.segment(curr_param_offset + e * pos_dim, pos_dim) = curr_pos.segment(0, pos_dim);
	}
	curr_param_offset += num_end_effs * pos_dim;

	int param_size = curr_param_offset - param_offset;
	assert(param_size == GetAMPObsPoseSize());

	return param_size;
}

int cSceneImitateAMP::RecordAMPObsVel(int agent_id, const Eigen::VectorXd& vel, const tQuaternion& ref_origin_rot,
										int param_offset, Eigen::VectorXd& out_data) const
{
	const int vel_dim = cKinTree::gVelDim;
	const int ang_vel_dim = cKinTree::gAngVelDim;

	const auto* character = GetAgentChar(agent_id);
	int curr_param_offset = param_offset;

	tVector root_vel = cKinTree::GetRootVel(vel);
	tVector root_ang_vel = cKinTree::GetRootAngVel(vel);
	if (mEnableAMPObsLocalRoot)
	{
		root_vel = cMathUtil::QuatRotVec(ref_origin_rot, root_vel);
		root_ang_vel = cMathUtil::QuatRotVec(ref_origin_rot, root_ang_vel);
	}
	out_data.segment(curr_param_offset, vel_dim) = root_vel.segment(0, vel_dim);
	out_data.segment(curr_param_offset + vel_dim, ang_vel_dim) = root_ang_vel.segment(0, ang_vel_dim);
	curr_param_offset += vel_dim + ang_vel_dim;

	int root_id = character->GetRootID();
	int vel_size = static_cast<int>(vel.size());
	int root_param_size = character->GetParamSize(root_id);
	int joint_vel_size = vel_size - root_param_size;
	out_data.segment(curr_param_offset, joint_vel_size) = vel.segment(root_param_size, joint_vel_size);
	curr_param_offset += joint_vel_size;

	int param_size = curr_param_offset - param_offset;
	assert(param_size == GetAMPObsVelSize());

	return param_size;
}

bool cSceneImitateAMP::EnableTestTimeWarp() const
{
	return mTimeWarper != nullptr;
}

int cSceneImitateAMP::GetTimeWarpDataDim() const
{
	const auto& sim_char = GetCharacter();
	int num_joints = sim_char->GetNumJoints();
	int dim = num_joints * cKinTree::gPosDim;
	return dim;
}

void cSceneImitateAMP::BuildTimeWarper()
{
	int data_dim = GetTimeWarpDataDim();

	const auto& sim_char = GetCharacter();
	const auto& ctrl = sim_char->GetController();
	const cCtController* ct_ctrl = dynamic_cast<const cCtController*>(ctrl.get());
	if (ct_ctrl == nullptr)
	{
		printf("Only ct controllers are supported for dynamic time warping\n");
		throw;
	}

	double update_rate = ct_ctrl->GetUpdateRate();
	double max_time = std::max(mTimerParams.mTimeMax, mTimerParamsEnd.mTimeMax);
	if (std::isfinite(max_time))
	{
		int buffer_size = int(std::ceil(update_rate * max_time)) + 2;

		mTimeWarper = std::unique_ptr<cDynamicTimeWarper>(new cDynamicTimeWarper());
		mTimeWarper->Init(data_dim, data_dim, buffer_size, TimeWarpCost);
		UpdateTimeWarper();
	}
}

void cSceneImitateAMP::ResetTimeWarper()
{
	mTimeWarper->Reset();
	UpdateTimeWarper();
}

void cSceneImitateAMP::UpdateTimeWarper()
{
	const auto& sim_char = GetCharacter();
	const auto& kin_char = GetKinChar();

	Eigen::VectorXd sim_data;
	BuildTimeWarpData(*sim_char, sim_data);
	mTimeWarper->AddSample0(sim_data);

	Eigen::VectorXd kin_data;
	BuildTimeWarpData(*kin_char, kin_data);
	mTimeWarper->AddSample1(kin_data);
}

void cSceneImitateAMP::BuildTimeWarpData(const cCharacter& character, Eigen::VectorXd& out_data) const
{
	int num_joints = character.GetNumJoints();
	out_data.resize(num_joints * cKinTree::gPosDim);

	tVector root_pos = character.GetRootPos();
	out_data(0) = 0.0;
	out_data(1) = root_pos[1];
	out_data(2) = 0.0;

	for (int j = 1; j < num_joints; ++j)
	{
		tVector joint_pos = character.CalcJointPos(j);
		tVector pos_delta = joint_pos - root_pos;
		out_data[j * cKinTree::gPosDim] = pos_delta[0];
		out_data[j * cKinTree::gPosDim + 1] = pos_delta[1];
		out_data[j * cKinTree::gPosDim + 2] = pos_delta[2];
	}
}