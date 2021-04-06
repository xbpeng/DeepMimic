#include "CtController.h"
#include "sim/SimCharacter.h"
#include "util/FileUtil.h"
#include "util/JsonUtil.h"

cCtController::cCtController() : cDeepMimicCharController()
{
	mUpdateRate = 30.0;
	mCyclePeriod = 1;
	mEnablePhaseInput = false;
	mRecordWorldRootPos = false;
	mRecordWorldRootRot = false;
	SetViewDistMax(1);

	mPhaseOffset = 0;
	mInitTimeOffset = 0;

	mActionSize = 0;
}

cCtController::~cCtController()
{
}

void cCtController::SetUpdateRate(double rate)
{
	mUpdateRate = rate;
}

double cCtController::GetUpdateRate() const
{
	return mUpdateRate;
}

void cCtController::UpdateCalcTau(double timestep, Eigen::VectorXd& out_tau)
{
	cDeepMimicCharController::UpdateCalcTau(timestep, out_tau);
	UpdateBuildTau(timestep, out_tau);
}

int cCtController::GetStateSize() const
{
	int state_size = cDeepMimicCharController::GetStateSize();
	state_size += GetStatePhaseSize();
	return state_size;
}

int cCtController::GetActionSize() const
{
	int a_size = mActionSize;
	return a_size;
}

void cCtController::BuildStateOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	cDeepMimicCharController::BuildStateOffsetScale(out_offset, out_scale);
	
	if (mEnablePhaseInput)
	{
		Eigen::VectorXd phase_offset;
		Eigen::VectorXd phase_scale;
		BuildStatePhaseOffsetScale(phase_offset, phase_scale);
		
		int phase_idx = GetStatePhaseOffset();
		int phase_size = GetStatePhaseSize();
		out_offset.segment(phase_idx, phase_size) = phase_offset;
		out_scale.segment(phase_idx, phase_size) = phase_scale;
	}
}

void cCtController::BuildActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	int action_size = GetActionSize();
	out_min = Eigen::VectorXd::Zero(action_size);
	out_max = Eigen::VectorXd::Zero(action_size);

	int root_id = mChar->GetRootID();
	int num_joints = mChar->GetNumJoints();

	for (int j = root_id + 1; j < num_joints; ++j)
	{
		const cSimBodyJoint& joint = mChar->GetJoint(j);
		if (joint.IsValid())
		{
			int param_offset = GetCtrlParamOffset(j);
			int param_size = GetCtrlParamSize(j);

			if (param_size > 0)
			{
				Eigen::VectorXd lim_min;
				Eigen::VectorXd lim_max;
				BuildJointActionBounds(j, lim_min, lim_max);
				assert(lim_min.size() == param_size);
				assert(lim_max.size() == param_size);

				out_min.segment(param_offset, param_size) = lim_min;
				out_max.segment(param_offset, param_size) = lim_max;
			}
		}
	}
}

void cCtController::ResetParams()
{
	cDeepMimicCharController::ResetParams();
	mPhaseOffset = 0;
	mInitTimeOffset = 0;
}

int cCtController::GetPosFeatureDim() const
{
	int pos_dim = cKinTree::gPosDim;
	return pos_dim;
}

int cCtController::GetRotFeatureDim() const
{
	int rot_dim = 2 * cKinTree::gPosDim;
	return rot_dim;
}

int cCtController::GetVelFeatureDim() const
{
	int vel_dim = cKinTree::gVelDim;
	return vel_dim;
}

int cCtController::GetAngVelFeatureDim() const
{
	int ang_vel_dim = cKinTree::gAngVelDim;
	return ang_vel_dim;
}

double cCtController::GetCyclePeriod() const
{
	return mCyclePeriod;
}

void cCtController::SetCyclePeriod(double period)
{
	mCyclePeriod = period;
}

void cCtController::SetInitTime(double time)
{
	mTime = time;
	mPrevActionTime = time;
	mPhaseOffset = 0;
	mInitTimeOffset = -mTime;
}

double cCtController::GetPhase() const
{
	double phase = mTime / mCyclePeriod;
	phase += mPhaseOffset;
	phase = std::fmod(phase, 1.0);
	phase = (phase < 0) ? (1 + phase) : phase;
	return phase;
}

bool cCtController::ParseParams(const Json::Value& json)
{
	bool succ = cDeepMimicCharController::ParseParams(json);

	mUpdateRate = json.get("QueryRate", mUpdateRate).asDouble();
	mCyclePeriod = json.get("CyclePeriod", mCyclePeriod).asDouble();
	mEnablePhaseInput = json.get("EnablePhaseInput", mEnablePhaseInput).asBool();
	mRecordWorldRootPos = json.get("RecordWorldRootPos", mRecordWorldRootPos).asBool();
	mRecordWorldRootRot = json.get("RecordWorldRootRot", mRecordWorldRootRot).asBool();

	return succ;
}

void cCtController::InitResources()
{
	int num_joints = mChar->GetNumJoints();
	BuildCtrlParamOffset(mCtrlParamOffset);
	mActionSize = GetCtrlParamOffset(num_joints - 1) + GetCtrlParamSize(num_joints - 1);

	cDeepMimicCharController::InitResources();
}

void cCtController::BuildCtrlParamOffset(Eigen::VectorXi& out_offset) const
{
	int num_joints = mChar->GetNumJoints();
	out_offset.resize(num_joints);

	int offset = GetActionCtrlOffset();
	for (int j = 0; j < num_joints; ++j)
	{
		int curr_size = GetCtrlParamSize(j);
		out_offset[j] = offset;
		offset += curr_size;
	}
}

void cCtController::UpdateBuildTau(double time_step, Eigen::VectorXd& out_tau)
{
	out_tau = Eigen::VectorXd::Zero(mChar->GetNumDof());
	
	int root_id = mChar->GetRootID();
	int root_size = mChar->GetParamSize(root_id);
	int num_joints = mChar->GetNumJoints();
	int ctrl_offset = GetActionCtrlOffset();

	for (int j = root_id + 1; j < num_joints; ++j)
	{
		int retarget_joint = RetargetJointID(j);
		int param_offset = mChar->GetParamOffset(j);
		int param_size = mChar->GetParamSize(j);
		int retarget_offset = mChar->GetParamOffset(retarget_joint);
		int retarget_size = mChar->GetParamSize(retarget_joint);
		assert(param_size == retarget_size);

		retarget_offset -= root_size;
		retarget_offset += ctrl_offset;
		out_tau.segment(param_offset, param_size) = mAction.segment(retarget_offset, retarget_size);
	}
}

bool cCtController::CheckNeedNewAction(double timestep) const
{
	double curr_time = mTime;
	curr_time += mInitTimeOffset;
	bool new_action = cMathUtil::CheckNextInterval(timestep, curr_time, 1 / mUpdateRate);
	return new_action;
}

void cCtController::NewActionUpdate()
{
	cDeepMimicCharController::NewActionUpdate();
}

void cCtController::BuildActionOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int action_size = GetActionSize();
	out_offset = Eigen::VectorXd::Zero(action_size);
	out_scale = Eigen::VectorXd::Ones(action_size);

	int root_id = mChar->GetRootID();
	int root_size = mChar->GetParamSize(mChar->GetRootID());
	int num_joints = mChar->GetNumJoints();
	int ctrl_offset = GetActionCtrlOffset();

	for (int j = root_id + 1; j < num_joints; ++j)
	{
		const cSimBodyJoint& joint = mChar->GetJoint(j);
		if (joint.IsValid())
		{
			int param_offset = GetCtrlParamOffset(j);
			int param_size = GetCtrlParamSize(j);

			if (param_size > 0)
			{
				Eigen::VectorXd curr_offset;
				Eigen::VectorXd curr_scale;
				BuildJointActionOffsetScale(j, curr_offset, curr_scale);
				assert(curr_offset.size() == param_size);
				assert(curr_scale.size() == param_size);

				out_offset.segment(param_offset, param_size) = curr_offset;
				out_scale.segment(param_offset, param_size) = curr_scale;
			}
		}
	}
}

void cCtController::BuildStateNormGroups(Eigen::VectorXi& out_groups) const
{
	cDeepMimicCharController::BuildStateNormGroups(out_groups);

	if (mEnablePhaseInput)
	{
		int phase_group = gNormGroupNone;
		int phase_offset = GetStatePhaseOffset();
		int phase_size = GetStatePhaseSize();
		out_groups.segment(phase_offset, phase_size) = phase_group * Eigen::VectorXi::Ones(phase_size);
	}
}

void cCtController::RecordState(Eigen::VectorXd& out_state)
{
	Eigen::VectorXd phase_state;
	cDeepMimicCharController::RecordState(out_state);

	if (mEnablePhaseInput)
	{
		int phase_offset = GetStatePhaseOffset();
		int phase_size = GetStatePhaseSize();
		BuildStatePhase(phase_state);
		out_state.segment(phase_offset, phase_size) = phase_state;
	}
}

std::string cCtController::GetName() const
{
	return "ct";
}

int cCtController::GetStatePoseSize() const
{
	int pos_dim = GetPosFeatureDim();
	int rot_dim = GetRotFeatureDim();
	int size = mChar->GetNumBodyParts() * (pos_dim + rot_dim) + 1; // +1 for root y
	return size;
}

int cCtController::GetStateVelSize() const
{
	int vel_dim = GetVelFeatureDim();
	int ang_vel_dim = GetAngVelFeatureDim();
	int size = mChar->GetNumBodyParts() * (vel_dim + ang_vel_dim);
	return size;
}

int cCtController::GetStatePoseOffset() const
{
	return cDeepMimicCharController::GetStatePoseOffset() + GetStatePhaseSize();
}

int cCtController::GetStatePhaseSize() const
{
	int phase_size = (mEnablePhaseInput) ? 1 : 0;
	return phase_size;
}

int cCtController::GetStatePhaseOffset() const
{
	return 0;
}

int cCtController::GetActionCtrlOffset() const
{
	return 0;
}


int cCtController::GetActionCtrlSize() const
{
	int ctrl_size = mActionSize;
	return ctrl_size;
}

int cCtController::GetCtrlParamOffset(int joint_id) const
{
	int param_offset = mCtrlParamOffset[joint_id];
	return param_offset;
}

int cCtController::GetCtrlParamSize(int joint_id) const
{
	int param_size = 0;
	int root_id = mChar->GetRootID();

	if (joint_id != root_id)
	{
		const auto& joint_mat = mChar->GetJointMat();
		param_size = cCtCtrlUtil::GetParamDimTorque(joint_mat, joint_id);
	}

	return param_size;
}

void cCtController::BuildStatePhaseOffsetScale(Eigen::VectorXd& phase_offset, Eigen::VectorXd& phase_scale) const
{
	double offset = -0.5;
	double scale = 2;
	int phase_size = GetStatePhaseSize();
	phase_offset = offset * Eigen::VectorXd::Ones(phase_size);
	phase_scale = scale * Eigen::VectorXd::Ones(phase_size);
}

void cCtController::BuildStatePose(Eigen::VectorXd& out_pose) const
{
	tMatrix origin_trans = mChar->BuildOriginTrans();
	tQuaternion origin_quat = cMathUtil::RotMatToQuaternion(origin_trans);

	tVector root_pos = mChar->GetRootPos();
	double ground_h = SampleGroundHeight(root_pos);

	tVector root_pos_rel = root_pos;
	root_pos_rel[1] -= ground_h;

	root_pos_rel[3] = 1;
	root_pos_rel = origin_trans * root_pos_rel;
	root_pos_rel[3] = 0;

	out_pose = Eigen::VectorXd::Zero(GetStatePoseSize());
	out_pose[0] = root_pos_rel[1];

	int num_parts = mChar->GetNumBodyParts();
	int root_id = mChar->GetRootID();

	int pos_dim = GetPosFeatureDim();
	int rot_dim = GetRotFeatureDim();

	for (int i = 0; i < num_parts; ++i)
	{
		int part_id = RetargetJointID(i);
		if (mChar->IsValidBodyPart(part_id))
		{
			const auto& curr_part = mChar->GetBodyPart(part_id);
			tVector curr_pos = curr_part->GetPos();
			curr_pos[1] -= ground_h;

			if (!mRecordWorldRootPos || i != root_id)
			{
				curr_pos[3] = 1;
				curr_pos = origin_trans * curr_pos;
				curr_pos -= root_pos_rel;
				curr_pos[3] = 0;
			}

			int pos_idx = (pos_dim + rot_dim) * i + 1;
			out_pose.segment(pos_idx, pos_dim) = curr_pos.segment(0, pos_dim);

			tQuaternion curr_quat = curr_part->GetRotation();
			if (!mRecordWorldRootRot || (i != root_id))
			{
				curr_quat = origin_quat * curr_quat;
			}

			tVector curr_rot_norm;
			tVector curr_rot_tan;
			cMathUtil::CalcNormalTangent(curr_quat, curr_rot_norm, curr_rot_tan);

			int rot_idx = (pos_dim + rot_dim) * i + 1 + pos_dim;
			out_pose.segment(rot_idx, rot_dim / 2) = curr_rot_norm.segment(0, rot_dim / 2);
			out_pose.segment(rot_idx + rot_dim / 2, rot_dim / 2) = curr_rot_tan.segment(0, rot_dim / 2);
		}
	}
}

void cCtController::BuildStateVel(Eigen::VectorXd& out_vel) const
{
	int num_parts = mChar->GetNumBodyParts();
	tMatrix origin_trans = mChar->BuildOriginTrans();
	tQuaternion origin_quat = cMathUtil::RotMatToQuaternion(origin_trans);

	int vel_dim = GetVelFeatureDim();
	int ang_vel_dim = GetAngVelFeatureDim();

	out_vel = Eigen::VectorXd::Zero(GetStateVelSize());

	int idx = 0;
	for (int i = 0; i < num_parts; ++i)
	{
		int part_id = RetargetJointID(i);
		int root_id = mChar->GetRootID();

		const auto& curr_part = mChar->GetBodyPart(part_id);
		tVector curr_vel = curr_part->GetLinearVelocity();

		if (!mRecordWorldRootRot || i != root_id)
		{
			curr_vel = origin_trans * curr_vel;
		}

		int vel_idx = (vel_dim + ang_vel_dim) * i;
		out_vel.segment(vel_idx, vel_dim) = curr_vel.segment(0, vel_dim);
		
		tVector curr_ang_vel = curr_part->GetAngularVelocity();
		if (!mRecordWorldRootRot || i != root_id)
		{
			curr_ang_vel = origin_trans * curr_ang_vel;
		}

		int ang_vel_idx = (vel_dim + ang_vel_dim) * i + vel_dim;
		out_vel.segment(ang_vel_idx, ang_vel_dim) = curr_ang_vel.segment(0, ang_vel_dim);
	}
}

void cCtController::BuildStatePhase(Eigen::VectorXd& out_phase) const
{
	double phase = GetPhase();
	out_phase = Eigen::VectorXd::Zero(GetStatePhaseSize());
	out_phase[0] = phase;
}

void cCtController::BuildJointActionBounds(int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	cCtCtrlUtil::BuildBoundsTorque(joint_mat, joint_id, out_min, out_max);
}

void cCtController::BuildJointActionOffsetScale(int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	cCtCtrlUtil::BuildOffsetScaleTorque(joint_mat, joint_id, out_offset, out_scale);
}

int cCtController::RetargetJointID(int joint_id) const
{
	return joint_id;
}