#include "CtController.h"
#include "sim/SimCharacter.h"
#include "util/FileUtil.h"
#include "util/JsonUtil.h"

cCtController::cCtController() : cDeepMimicCharController()
{
	mUpdateRate = 30.0;
	mCyclePeriod = 1;
	mEnablePhaseInput = false;
	mEnablePhaseAction = false;
	mRecordWorldRootPos = false;
	mRecordWorldRootRot = false;
	SetViewDistMax(1);

	mPhaseOffset = 0;
	mInitTimeOffset = 0;
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
	// use unnormalized phase
	double prev_phase = 0;
	if (mEnablePhaseAction)
	{
		prev_phase = mTime / mCyclePeriod;
		prev_phase += mPhaseOffset;
	}

	cDeepMimicCharController::UpdateCalcTau(timestep, out_tau);

	if (mEnablePhaseAction)
	{
		double phase_rate = GetPhaseRate();
		double tar_phase = prev_phase + timestep * phase_rate;
		mPhaseOffset = tar_phase - mTime / mCyclePeriod;
		mPhaseOffset = std::fmod(mPhaseOffset, 1.0);
	}

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
	int a_size = 0;
	a_size += GetActionPhaseSize();
	a_size += GetActionCtrlSize();
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
	int root_size = mChar->GetParamSize(root_id);
	int num_joints = mChar->GetNumJoints();
	int ctrl_offset = GetActionCtrlOffset();

	if (mEnablePhaseAction)
	{
		int phase_offset = GetActionPhaseOffset();
		out_min[phase_offset] = -5;
		out_max[phase_offset] = 5;
	}

	for (int j = root_id + 1; j < num_joints; ++j)
	{
		const cSimBodyJoint& joint = mChar->GetJoint(j);
		if (joint.IsValid())
		{
			int param_offset = mChar->GetParamOffset(j);
			int param_size = mChar->GetParamSize(j);

			if (param_size > 0)
			{
				Eigen::VectorXd lim_min;
				Eigen::VectorXd lim_max;
				BuildJointActionBounds(j, lim_min, lim_max);
				assert(lim_min.size() == param_size);
				assert(lim_max.size() == param_size);

				param_offset -= root_size;
				param_offset += ctrl_offset;
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
	int rot_dim = cKinTree::gRotDim;
	return rot_dim;
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
	mEnablePhaseAction = json.get("EnablePhaseAction", mEnablePhaseAction).asBool();
	mRecordWorldRootPos = json.get("RecordWorldRootPos", mRecordWorldRootPos).asBool();
	mRecordWorldRootRot = json.get("RecordWorldRootRot", mRecordWorldRootRot).asBool();

	return succ;
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

	if (mEnablePhaseAction)
	{
		int phase_offset = GetActionPhaseOffset();
		out_offset[phase_offset] = -1 / mCyclePeriod;
		out_scale[phase_offset] = 1;
	}

	for (int j = root_id + 1; j < num_joints; ++j)
	{
		const cSimBodyJoint& joint = mChar->GetJoint(j);
		if (joint.IsValid())
		{
			int param_offset = mChar->GetParamOffset(j);
			int param_size = mChar->GetParamSize(j);

			if (param_size > 0)
			{
				Eigen::VectorXd curr_offset;
				Eigen::VectorXd curr_scale;
				BuildJointActionOffsetScale(j, curr_offset, curr_scale);
				assert(curr_offset.size() == param_size);
				assert(curr_scale.size() == param_size);

				param_offset -= root_size;
				param_offset += ctrl_offset;
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
	int pos_dim = GetPosFeatureDim();
	int rot_dim = GetRotFeatureDim();
	int size = mChar->GetNumBodyParts() * (pos_dim + rot_dim - 1);
	return size;
}

int cCtController::GetStatePoseOffset() const
{
	return cDeepMimicCharController::GetStatePoseOffset() + GetStatePhaseSize();
}

int cCtController::GetStatePhaseSize() const
{
	return (mEnablePhaseInput) ? 1 : 0;
}

int cCtController::GetStatePhaseOffset() const
{
	return 0;
}

int cCtController::GetActionPhaseOffset() const
{
	return 0;
}

int cCtController::GetActionPhaseSize() const
{
	return (mEnablePhaseAction) ? 1 : 0;
}

int cCtController::GetActionCtrlOffset() const
{
	return GetActionPhaseSize();
}

int cCtController::GetActionCtrlSize() const
{
	int ctrl_size = mChar->GetNumDof();
	int root_size = mChar->GetParamSize(mChar->GetRootID());
	ctrl_size -= root_size;
	return ctrl_size;
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

	bool flip_stance = FlipStance();
	if (flip_stance)
	{
		origin_trans.row(2) *= -1; // reflect z
	}

	tVector root_pos = mChar->GetRootPos();
	tVector root_pos_rel = root_pos;

	root_pos_rel[3] = 1;
	root_pos_rel = origin_trans * root_pos_rel;
	root_pos_rel[3] = 0;

	out_pose = Eigen::VectorXd::Zero(GetStatePoseSize());
	out_pose[0] = root_pos_rel[1];
	int num_parts = mChar->GetNumBodyParts();
	int root_id = mChar->GetRootID();

	int pos_dim = GetPosFeatureDim();
	int rot_dim = GetRotFeatureDim();

	tQuaternion mirror_inv_origin_quat = origin_quat.conjugate();
	mirror_inv_origin_quat = cMathUtil::MirrorQuaternion(mirror_inv_origin_quat, cMathUtil::eAxisZ);

	int idx = 1;
	for (int i = 0; i < num_parts; ++i)
	{
		int part_id = RetargetJointID(i);
		if (mChar->IsValidBodyPart(part_id))
		{
			const auto& curr_part = mChar->GetBodyPart(part_id);
			tVector curr_pos = curr_part->GetPos();

			if (mRecordWorldRootPos && i == root_id)
			{
				if (flip_stance)
				{
					curr_pos = cMathUtil::QuatRotVec(origin_quat, curr_pos);
					curr_pos[2] = -curr_pos[2];
					curr_pos = cMathUtil::QuatRotVec(mirror_inv_origin_quat, curr_pos);
				}
			}
			else
			{
				curr_pos[3] = 1;
				curr_pos = origin_trans * curr_pos;
				curr_pos -= root_pos_rel;
				curr_pos[3] = 0;
			}

			out_pose.segment(idx, pos_dim) = curr_pos.segment(0, pos_dim);
			idx += pos_dim;

			tQuaternion curr_quat = curr_part->GetRotation();
			if (mRecordWorldRootRot && i == root_id)
			{
				if (flip_stance)
				{
					curr_quat = origin_quat * curr_quat;
					curr_quat = cMathUtil::MirrorQuaternion(curr_quat, cMathUtil::eAxisZ);
					curr_quat = mirror_inv_origin_quat * curr_quat;
				}
			}
			else
			{
				curr_quat = origin_quat * curr_quat;
				if (flip_stance)
				{
					curr_quat = cMathUtil::MirrorQuaternion(curr_quat, cMathUtil::eAxisZ);
				}
			}

			if (curr_quat.w() < 0)
			{
				curr_quat.w() *= -1;
				curr_quat.x() *= -1;
				curr_quat.y() *= -1;
				curr_quat.z() *= -1;
			}
			out_pose.segment(idx, rot_dim) = cMathUtil::QuatToVec(curr_quat).segment(0, rot_dim);
			idx += rot_dim;
		}
	}
}

void cCtController::BuildStateVel(Eigen::VectorXd& out_vel) const
{
	int num_parts = mChar->GetNumBodyParts();
	tMatrix origin_trans = mChar->BuildOriginTrans();
	tQuaternion origin_quat = cMathUtil::RotMatToQuaternion(origin_trans);

	bool flip_stance = FlipStance();
	if (flip_stance)
	{
		origin_trans.row(2) *= -1; // reflect z
	}

	int pos_dim = GetPosFeatureDim();
	int rot_dim = GetRotFeatureDim();

	out_vel = Eigen::VectorXd::Zero(GetStateVelSize());

	tQuaternion mirror_inv_origin_quat = origin_quat.conjugate();
	mirror_inv_origin_quat = cMathUtil::MirrorQuaternion(mirror_inv_origin_quat, cMathUtil::eAxisZ);
	
	int idx = 0;
	for (int i = 0; i < num_parts; ++i)
	{
		int part_id = RetargetJointID(i);
		int root_id = mChar->GetRootID();

		const auto& curr_part = mChar->GetBodyPart(part_id);
		tVector curr_vel = curr_part->GetLinearVelocity();

		if (mRecordWorldRootRot && i == root_id)
		{
			if (flip_stance)
			{
				curr_vel = cMathUtil::QuatRotVec(origin_quat, curr_vel);
				curr_vel[2] = -curr_vel[2];
				curr_vel = cMathUtil::QuatRotVec(mirror_inv_origin_quat, curr_vel);
			}
		}
		else
		{
			curr_vel = origin_trans * curr_vel;
		}

		out_vel.segment(idx, pos_dim) = curr_vel.segment(0, pos_dim);
		idx += pos_dim;

		tVector curr_ang_vel = curr_part->GetAngularVelocity();
		if (mRecordWorldRootRot && i == root_id)
		{
			if (flip_stance)
			{
				curr_ang_vel = cMathUtil::QuatRotVec(origin_quat, curr_ang_vel);
				curr_ang_vel[2] = -curr_ang_vel[2];
				curr_ang_vel = -curr_ang_vel;
				curr_ang_vel = cMathUtil::QuatRotVec(mirror_inv_origin_quat, curr_ang_vel);
			}
		}
		else
		{
			curr_ang_vel = origin_trans * curr_ang_vel;
			if (flip_stance)
			{
				curr_ang_vel = -curr_ang_vel;
			}
		}

		out_vel.segment(idx, rot_dim - 1) = curr_ang_vel.segment(0, rot_dim - 1);
		idx += rot_dim - 1;
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

bool cCtController::FlipStance() const
{
	return false;
}

int cCtController::RetargetJointID(int joint_id) const
{
	return joint_id;
}

double cCtController::GetPhaseRate() const
{
	assert(mEnablePhaseAction);
	double phase_rate = 0;
	if (mEnablePhaseAction)
	{
		int phase_offset = GetActionPhaseOffset();
		phase_rate = mAction[phase_offset];
	}
	return phase_rate;
}