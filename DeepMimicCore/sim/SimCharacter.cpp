#include "SimCharacter.h"
#include <iostream>

#include "SimBox.h"
#include "SimCapsule.h"
#include "SimSphere.h"
#include "RBDUtil.h"

#include "util/FileUtil.h"
#include "util/JsonUtil.h"

cSimCharacter::tParams::tParams()
{
	mID = gInvalidIdx;
	mCharFile = "";
	mStateFile = "";
	mInitPos = tVector(0, 0, 0, 0);
	mLoadDrawShapes = true;

	mEnableContactFall = true;
}


cSimCharacter::cSimCharacter()
{
	mFriction = 0.9;
	mEnableContactFall = true;
}

cSimCharacter::~cSimCharacter()
{
	RemoveFromWorld();
}

bool cSimCharacter::Init(const std::shared_ptr<cWorld>& world, const tParams& params)
{
	bool succ = true;
	bool succ_skeleton = cCharacter::Init(params.mCharFile, params.mLoadDrawShapes);
	succ &= succ_skeleton;

	SetID(params.mID);
	RemoveFromWorld();
	mWorld = world;
	mEnableContactFall = params.mEnableContactFall;

	if (succ_skeleton)
	{
		succ &= BuildSimBody(params);
	}

	if (params.mStateFile != "")
	{
		bool succ_state = ReadState(params.mStateFile);

		if (!succ_state)
		{
			printf("Failed to load character state from %s\n", params.mStateFile.c_str());
		}
	}

	if (succ)
	{
		mPose0 = mPose;
		mVel0 = mVel;

		SetPose(mPose);
		SetVel(mVel);
	}

	return succ;
}

void cSimCharacter::Clear()
{
	cCharacter::Clear();

	RemoveFromWorld();
	mBodyDefs.resize(0, 0);

	if (HasController())
	{
		mController->Clear();
	}
}


void cSimCharacter::Reset()
{
	cCharacter::Reset();
	if (HasController())
	{
		mController->Reset();
	}
	
	ClearJointTorques();
}


void cSimCharacter::Update(double timestep)
{
	ClearJointTorques();

	if (HasController())
	{
		mController->Update(timestep);
	}

	// dont clear torques until next frame since they can be useful for visualization
	UpdateJoints();
}

void cSimCharacter::PostUpdate(double time_step)
{
	UpdateLinkVel();
	BuildPose(mPose);
	BuildVel(mVel);

	if (HasController())
	{
		mController->PostUpdate(time_step);
	}
}

tVector cSimCharacter::GetRootPos() const
{
	int root_id = GetRootID();
	const cSimBodyJoint& root = GetJoint(root_id);
	tVector pos = root.CalcWorldPos();
	return pos;
}

void cSimCharacter::GetRootRotation(tVector& out_axis, double& out_theta) const
{
	tQuaternion rot = GetRootRotation();
	cMathUtil::QuaternionToAxisAngle(rot, out_axis, out_theta);
}

tQuaternion cSimCharacter::GetRootRotation() const
{
	int root_id = GetRootID();
	const cSimBodyJoint& root = GetJoint(root_id);
	tQuaternion rot = root.CalcWorldRotation();
	rot = mInvRootAttachRot * rot;
	return rot;
}

tVector cSimCharacter::GetRootVel() const
{
	int root_id = GetRootID();
	const cSimBodyJoint& root = GetJoint(root_id);
	tVector vel = root.CalcWorldVel();
	return vel;
}

tVector cSimCharacter::GetRootAngVel() const
{
	int root_id = GetRootID();
	const cSimBodyJoint& root = GetJoint(root_id);
	tVector ang_vel = root.CalcWorldAngVel();
	return ang_vel;
}

const Eigen::MatrixXd& cSimCharacter::GetBodyDefs() const
{
	return mBodyDefs;
}

void cSimCharacter::SetRootPos(const tVector& pos)
{
	cCharacter::SetRootPos(pos);
	SetPose(mPose);
}

void cSimCharacter::SetRootRotation(const tVector& axis, double theta)
{
	tQuaternion q = cMathUtil::AxisAngleToQuaternion(axis, theta);
	SetRootTransform(GetRootPos(), q);
}

void cSimCharacter::SetRootRotation(const tQuaternion& q)
{
	SetRootTransform(GetRootPos(), q);
}

void cSimCharacter::SetRootTransform(const tVector& pos, const tQuaternion& rot)
{
	tQuaternion root_rot = cKinTree::GetRootRot(mPose);
	tVector root_vel = cKinTree::GetRootVel(mVel);
	tVector root_ang_vel = cKinTree::GetRootAngVel(mVel);
	tQuaternion delta_rot = rot * root_rot.inverse();

	root_vel = cMathUtil::QuatRotVec(delta_rot, root_vel);
	root_ang_vel = cMathUtil::QuatRotVec(delta_rot, root_ang_vel);

	cKinTree::SetRootPos(pos, mPose);
	cKinTree::SetRootRot(rot, mPose);
	cKinTree::SetRootVel(root_vel, mVel);
	cKinTree::SetRootAngVel(root_ang_vel, mVel);

	SetPose(mPose);
	SetVel(mVel);
}

void cSimCharacter::SetRootVel(const tVector& vel)
{
	cCharacter::SetRootVel(vel);
	SetVel(mVel);
}

void cSimCharacter::SetRootAngVel(const tVector& ang_vel)
{
	cCharacter::SetRootAngVel(ang_vel);
	SetVel(mVel);
}

tQuaternion cSimCharacter::CalcHeadingRot() const
{
	tVector ref_dir = tVector(1, 0, 0, 0);
	tQuaternion root_rot = GetRootRotation();
	tVector rot_dir = cMathUtil::QuatRotVec(root_rot, ref_dir);
	double heading = std::atan2(-rot_dir[2], rot_dir[0]);
	return cMathUtil::AxisAngleToQuaternion(tVector(0, 1, 0, 0), heading);
}

int cSimCharacter::GetNumBodyParts() const
{
	return static_cast<int>(mBodyParts.size());
}

void cSimCharacter::SetVel(const Eigen::VectorXd& vel)
{
	cCharacter::SetVel(vel);

	double world_scale = mWorld->GetScale();
	int root_id = cKinTree::GetRootID();
	tVector root_vel = cKinTree::GetRootVel(vel);
	tVector root_ang_vel = cKinTree::GetRootAngVel(vel);
	cKinTree::eJointType root_type = cKinTree::GetJointType(mJointMat, root_id);

	if (!mMultBody->hasFixedBase())
	{
		mMultBody->setBaseVel(world_scale * btVector3(root_vel[0], root_vel[1], root_vel[2]));
		mMultBody->setBaseOmega(btVector3(root_ang_vel[0], root_ang_vel[1], root_ang_vel[2]));
	}
	else
	{
		cSimBodyJoint& root_joint = GetJoint(root_id);
		tQuaternion world_to_root = root_joint.CalcWorldRotation().inverse();
		tVector local_root_vel = cMathUtil::QuatRotVec(world_to_root, root_vel);
		tVector local_root_ang_vel = cMathUtil::QuatRotVec(world_to_root, root_ang_vel);
		Eigen::VectorXd root_params = Eigen::VectorXd::Zero(GetParamSize(root_id));

		switch (root_type)
		{
		case cKinTree::eJointTypeRevolute:
		{
			root_params[0] = local_root_ang_vel[2];
			break;
		}
		case cKinTree::eJointTypePlanar:
		{
			root_params[0] = local_root_vel[0];
			root_params[1] = local_root_vel[1];
			root_params[2] = local_root_ang_vel[2];
			break;
		}
		case cKinTree::eJointTypePrismatic:
		{
			root_params[0] = local_root_vel[0];
			break;
		}
		case cKinTree::eJointTypeFixed:
		{
			break;
		}
		case cKinTree::eJointTypeSpherical:
		{
			root_params[0] = local_root_ang_vel[0];
			root_params[1] = local_root_ang_vel[1];
			root_params[2] = local_root_ang_vel[2];
			break;
		}
		default:
			assert(false); // unsupported joint type
			break;
		}

		root_joint.SetVel(root_params);
	}

	for (int j = 1; j < GetNumJoints(); ++j)
	{
		cSimBodyJoint& curr_jont = GetJoint(j);
		int param_offset = GetParamOffset(j);
		int param_size = GetParamSize(j);
		Eigen::VectorXd curr_params = vel.segment(param_offset, param_size);
		curr_jont.SetVel(curr_params);
	}

	UpdateLinkVel();

	if (HasController())
	{
		mController->HandleVelReset();
	}
}

tVector cSimCharacter::CalcJointPos(int joint_id) const
{
	const cSimBodyJoint& joint = GetJoint(joint_id);
	tVector pos;
	
	if (joint.IsValid())
	{
		pos = joint.CalcWorldPos();
	}
	else
	{
		int parent_id = cKinTree::GetParent(mJointMat, joint_id);
		assert(parent_id != cKinTree::gInvalidJointID);
		assert(IsValidBodyPart(parent_id));

		tVector attach_pt = cKinTree::GetAttachPt(mJointMat, joint_id);
		tVector part_attach_pt = cKinTree::GetBodyAttachPt(mBodyDefs, parent_id);
		attach_pt -= part_attach_pt;

		const auto& parent_part = GetBodyPart(parent_id);
		pos = parent_part->LocalToWorldPos(attach_pt);
	}
	return pos;
}

tVector cSimCharacter::CalcJointVel(int joint_id) const
{
	const cSimBodyJoint& joint = GetJoint(joint_id);
	tVector vel;

	if (joint.IsValid())
	{
		vel = joint.CalcWorldVel();
	}
	else
	{
		int parent_id = cKinTree::GetParent(mJointMat, joint_id);
		assert(parent_id != cKinTree::gInvalidJointID);
		assert(IsValidBodyPart(parent_id));

		tVector attach_pt = cKinTree::GetAttachPt(mJointMat, joint_id);
		tVector part_attach_pt = cKinTree::GetBodyAttachPt(mBodyDefs, parent_id);
		attach_pt -= part_attach_pt;

		const auto& parent_part = GetBodyPart(parent_id);
		vel = parent_part->GetLinearVelocity(attach_pt);
	}

	return vel;
}

void cSimCharacter::CalcJointWorldRotation(int joint_id, tVector& out_axis, double& out_theta) const
{
	const auto& joint = GetJoint(joint_id);
	if (joint.IsValid())
	{
		joint.CalcWorldRotation(out_axis, out_theta);
	}
	else
	{
		int parent_id = cKinTree::GetParent(mJointMat, joint_id);
		assert(parent_id != cKinTree::gInvalidJointID);
		assert(IsValidBodyPart(parent_id));

		const auto& parent_part = GetBodyPart(parent_id);
		parent_part->GetRotation(out_axis, out_theta);
	}
}

tQuaternion cSimCharacter::CalcJointWorldRotation(int joint_id) const
{
	tQuaternion rot = tQuaternion::Identity();
	const auto& joint = GetJoint(joint_id);
	if (joint.IsValid())
	{
		rot = joint.CalcWorldRotation();
	}
	else
	{
		int parent_id = cKinTree::GetParent(mJointMat, joint_id);
		assert(parent_id != cKinTree::gInvalidJointID);
		assert(IsValidBodyPart(parent_id));

		const auto& parent_part = GetBodyPart(parent_id);
		rot = parent_part->GetRotation();
	}

	return rot;
}

tVector cSimCharacter::CalcCOM() const
{
	tVector com = tVector::Zero();
	double total_mass = 0;
	for (int i = 0; i < static_cast<int>(mBodyParts.size()); ++i)
	{
		if (IsValidBodyPart(i))
		{
			const auto& part = mBodyParts[i];
			double mass = part->GetMass();
			tVector curr_com = part->GetPos();

			com += mass * curr_com;
			total_mass += mass;
		}
	}
	com /= total_mass;
	return com;
}

tVector cSimCharacter::CalcCOMVel() const
{
	tVector com_vel = tVector::Zero();
	double total_mass = 0;
	for (int i = 0; i < static_cast<int>(mBodyParts.size()); ++i)
	{
		if (IsValidBodyPart(i))
		{
			const auto& part = mBodyParts[i];
			double mass = part->GetMass();
			tVector curr_vel = part->GetLinearVelocity();

			com_vel += mass * curr_vel;
			total_mass += mass;
		}
	}
	com_vel /= total_mass;
	return com_vel;
}

void cSimCharacter::CalcAABB(tVector& out_min, tVector& out_max) const
{
	out_min[0] = std::numeric_limits<double>::infinity();
	out_min[1] = std::numeric_limits<double>::infinity();
	out_min[2] = std::numeric_limits<double>::infinity();

	out_max[0] = -std::numeric_limits<double>::infinity();
	out_max[1] = -std::numeric_limits<double>::infinity();
	out_max[2] = -std::numeric_limits<double>::infinity();

	for (int i = 0; i < GetNumBodyParts(); ++i)
	{
		if (IsValidBodyPart(i))
		{
			const auto& part = GetBodyPart(i);

			tVector curr_min = tVector::Zero();
			tVector curr_max = tVector::Zero();
			part->CalcAABB(curr_min, curr_max);

			out_min = out_min.cwiseMin(curr_min);
			out_max = out_max.cwiseMax(curr_max);
		}
	}
}

tVector cSimCharacter::GetSize() const
{
	tVector aabb_min;
	tVector aabb_max;
	CalcAABB(aabb_min, aabb_max);
	return aabb_max - aabb_min;
}

const cSimBodyJoint& cSimCharacter::GetJoint(int joint_id) const
{
	return mJoints[joint_id];
}

cSimBodyJoint& cSimCharacter::GetJoint(int joint_id)
{
	return mJoints[joint_id];
}

const std::shared_ptr<cSimBodyLink>& cSimCharacter::GetBodyPart(int idx) const
{
	return mBodyParts[idx];
}

std::shared_ptr<cSimBodyLink>& cSimCharacter::GetBodyPart(int idx)
{
	return mBodyParts[idx];
}

tVector cSimCharacter::GetBodyPartPos(int idx) const
{
	auto& part = GetBodyPart(idx);
	tVector pos = part->GetPos();
	return pos;
}

tVector cSimCharacter::GetBodyPartVel(int idx) const
{
	auto& part = GetBodyPart(idx);
	tVector vel = part->GetLinearVelocity();
	return vel;
}

const std::shared_ptr<cSimBodyLink>& cSimCharacter::GetRootPart() const
{
	int root_idx = GetRootID();
	return mBodyParts[root_idx];
}

std::shared_ptr<cSimBodyLink> cSimCharacter::GetRootPart()
{
	int root_idx = GetRootID();
	return mBodyParts[root_idx];
}

void cSimCharacter::RegisterContacts(int contact_flags, int filter_flags)
{
	for (int i = 0; i < static_cast<int>(mBodyParts.size()); ++i)
	{
		if (IsValidBodyPart(i))
		{
			std::shared_ptr<cSimBodyLink>& part = mBodyParts[i];
			part->RegisterContact(contact_flags, filter_flags);
		}
	}
}

void cSimCharacter::UpdateContact(int contact_flags, int filter_flags)
{
	for (int i = 0; i < static_cast<int>(mBodyParts.size()); ++i)
	{
		if (IsValidBodyPart(i))
		{
			std::shared_ptr<cSimBodyLink>& part = mBodyParts[i];
			part->UpdateContact(contact_flags, filter_flags);
		}
	}
}

bool cSimCharacter::HasFallen() const
{
	bool fallen = false;
	if (mEnableContactFall)
	{
		fallen |= CheckFallContact();
	}
	return fallen;
}

bool cSimCharacter::HasStumbled() const
{
	bool stumbled = false;
	for (int j = 0; j < GetNumJoints(); ++j)
	{
		if (!IsEndEffector(j))
		{
			const auto& curr_part = GetBodyPart(j);
			bool contact = curr_part->IsInContact();
			if (contact)
			{
				stumbled = true;
				break;
			}
		}
	}
	return stumbled;
}

bool cSimCharacter::HasVelExploded(double vel_threshold /*= 100.0*/) const
{
	int num_bodies = GetNumBodyParts();
	for (int b = 0; b < num_bodies; ++b)
	{
		const auto& body = GetBodyPart(b);
		tVector vel = body->GetLinearVelocity();
		tVector ang_vel = body->GetAngularVelocity();
		double max_val = std::max(vel.cwiseAbs().maxCoeff(), ang_vel.cwiseAbs().maxCoeff());
		if (max_val > vel_threshold)
		{
			return true;
		}
	}
	return false;
}

bool cSimCharacter::IsInContact() const
{
	for (int i = 0; i < GetNumBodyParts(); ++i)
	{
		if (IsValidBodyPart(i))
		{
			if (IsInContact(i))
			{
				return true;
			}
		}
	}
	return false;
}

bool cSimCharacter::IsInContact(int idx) const
{
	return GetBodyPart(idx)->IsInContact();
}

const tEigenArr<cContactManager::tContactPt>& cSimCharacter::GetContactPts(int idx) const
{
	return GetBodyPart(idx)->GetContactPts();
}

const tEigenArr<cContactManager::tContactPt>& cSimCharacter::GetContactPts() const
{
	return GetContactPts(GetRootID());
}

const cContactManager::tContactHandle& cSimCharacter::GetContactHandle() const
{
	return GetRootPart()->GetContactHandle();
}

void cSimCharacter::SetController(std::shared_ptr<cCharController> ctrl)
{
	RemoveController();
	mController = ctrl;
}

void cSimCharacter::RemoveController()
{
	if (HasController())
	{
		mController.reset();
	}
}

bool cSimCharacter::HasController() const
{
	return mController != nullptr;
}

const std::shared_ptr<cCharController>& cSimCharacter::GetController()
{
	return mController;
}

const std::shared_ptr<cCharController>& cSimCharacter::GetController() const
{
	return mController;
}

void cSimCharacter::EnableController(bool enable)
{
	if (HasController())
	{
		mController->SetActive(enable);
	}
}

void cSimCharacter::ApplyForce(const tVector& force)
{
	ApplyForce(force, tVector::Zero());
}

void cSimCharacter::ApplyForce(const tVector& force, const tVector& local_pos)
{
	const auto& root_body = GetRootPart();
	const auto& joint = GetJoint(GetRootID());

	tVector world_pos = local_pos;
	world_pos[3] = 1;
	tMatrix joint_to_world = joint.BuildWorldTrans();
	world_pos = joint_to_world * world_pos;

	tVector body_local = root_body->WorldToLocalPos(world_pos);
	root_body->ApplyForce(force, body_local);
}

void cSimCharacter::ApplyTorque(const tVector& torque)
{
	const auto& root_body = GetRootPart();
	root_body->ApplyTorque(torque);
}

void cSimCharacter::ClearForces()
{
	int num_parts = GetNumBodyParts();
	for (int b = 0; b < num_parts; ++b)
	{
		if (IsValidBodyPart(b))
		{
			auto& part = GetBodyPart(b);
			part->ClearForces();
		}
	}
}

void cSimCharacter::ApplyControlForces(const Eigen::VectorXd& tau)
{
	assert(tau.size() == GetNumDof());
	for (int j = 1; j < GetNumJoints(); ++j)
	{
		cSimBodyJoint& joint = GetJoint(j);
		if (joint.IsValid())
		{
			int param_offset = GetParamOffset(j);
			int param_size = GetParamSize(j);
			if (param_size > 0)
			{
				Eigen::VectorXd curr_tau = tau.segment(param_offset, param_size);
				joint.AddTau(curr_tau);
			}
		}
	}
}

void cSimCharacter::PlayPossum()
{
	if (HasController())
	{
		mController->SetMode(cController::eModePassive);
	}
}

void cSimCharacter::SetPose(const Eigen::VectorXd& pose)
{
	cCharacter::SetPose(pose);

	double world_scale = mWorld->GetScale();
	int root_id = cKinTree::GetRootID();
	tVector root_pos = cKinTree::GetRootPos(pose);
	tQuaternion root_rot = cKinTree::GetRootRot(pose);
	cKinTree::eJointType root_type = cKinTree::GetJointType(mJointMat, root_id);

	tVector euler = cMathUtil::QuaternionToEuler(root_rot);
	mMultBody->setBasePos(world_scale * btVector3(root_pos[0], root_pos[1], root_pos[2]));
	mMultBody->setWorldToBaseRot(btQuaternion(root_rot.x(), root_rot.y(), root_rot.z(), root_rot.w()).inverse());

	cSimBodyJoint& root_joint = GetJoint(root_id);
	Eigen::VectorXd root_pose = Eigen::VectorXd::Zero(GetParamSize(root_id));
	if (root_type == cKinTree::eJointTypeSpherical)
	{
		root_pose = cMathUtil::QuatToVec(tQuaternion::Identity());
	}
	root_joint.SetPose(root_pose);

	for (int j = 1; j < GetNumJoints(); ++j)
	{
		cSimBodyJoint& curr_joint = GetJoint(j);
		int param_offset = GetParamOffset(j);
		int param_size = GetParamSize(j);
		Eigen::VectorXd curr_params = pose.segment(param_offset, param_size);
		curr_joint.SetPose(curr_params);
	}

	UpdateLinkPos();
	UpdateLinkVel();

	if (HasController())
	{
		mController->HandlePoseReset();
	}
}

bool cSimCharacter::BuildSimBody(const tParams& params)
{
	bool succ = true;
	succ = LoadBodyDefs(params.mCharFile, mBodyDefs);

	if (succ)
	{
		mInvRootAttachRot = cMathUtil::EulerToQuaternion(cKinTree::GetAttachTheta(mJointMat, GetRootID())).inverse();
		
		succ &= BuildMultiBody(mMultBody);
		succ &= BuildConstraints(mMultBody);
		succ &= BuildBodyLinks();
		succ &= BuildJoints();

		mWorld->AddCharacter(*this);

		mVecBuffer0.resize(mMultBody->getNumLinks() + 1);
		mVecBuffer1.resize(mMultBody->getNumLinks() + 1);
		mRotBuffer.resize(mMultBody->getNumLinks() + 1);
	}

	return succ;
}

bool cSimCharacter::BuildMultiBody(std::shared_ptr<cMultiBody>& out_body)
{
	bool succ = true;

	double world_scale = mWorld->GetScale();
	int num_joints = GetNumJoints();
	bool fixed_base = FixedBase();
	btVector3 base_intertia = btVector3(0, 0, 0);
	btScalar base_mass = 0;
	mMultBody = std::shared_ptr<cMultiBody>(new cMultiBody(num_joints, base_mass, base_intertia, fixed_base, false));

	btTransform base_trans;
	base_trans.setIdentity();
	mMultBody->setBaseWorldTransform(base_trans);

	for (int j = 0; j < num_joints; ++j)
	{
		cShape::eShape body_shape = cKinTree::GetBodyShape(mBodyDefs, j);
		tVector body_size = cKinTree::GetBodySize(mBodyDefs, j);
		double mass = cKinTree::GetBodyMass(mBodyDefs, j);
		cKinTree::eJointType joint_type = cKinTree::GetJointType(mJointMat, j);
		int parent_joint = cKinTree::GetParent(mJointMat, j);

		bool is_root = cKinTree::IsRoot(mJointMat, j);
		
		btCollisionShape* col_shape = BuildCollisionShape(body_shape, body_size);
		btVector3 inertia = btVector3(0, 0, 0);
		col_shape->calculateLocalInertia(static_cast<btScalar>(mass), inertia);

		// arg so many transforms...
		tQuaternion this_to_parent = cMathUtil::EulerToQuaternion(cKinTree::GetAttachTheta(mJointMat, j));
		tQuaternion body_to_this = cMathUtil::EulerToQuaternion(cKinTree::GetBodyAttachTheta(mBodyDefs, j));
		tQuaternion this_to_body = body_to_this.inverse();

		tQuaternion parent_body_to_parent = tQuaternion::Identity();
		if (parent_joint != gInvalidIdx)
		{
			parent_body_to_parent = cMathUtil::EulerToQuaternion(cKinTree::GetBodyAttachTheta(mBodyDefs, parent_joint));
		}
		tQuaternion parent_to_parent_body = parent_body_to_parent.inverse();

		tQuaternion body_to_parent_body = parent_to_parent_body * this_to_parent * body_to_this;
		tQuaternion parent_body_to_body = body_to_parent_body.inverse();

		// parent body attachment point in body's coordinate frame
		tVector parent_body_attach_pt = tVector::Zero();
		if (parent_joint != gInvalidIdx)
		{
			parent_body_attach_pt = cKinTree::GetBodyAttachPt(mBodyDefs, parent_joint);
		}
		parent_body_attach_pt = cMathUtil::QuatRotVec(parent_to_parent_body, parent_body_attach_pt);

		tVector joint_attach_pt = cKinTree::GetAttachPt(mJointMat, j);
		tVector body_attach_pt = cKinTree::GetBodyAttachPt(mBodyDefs, j);
		joint_attach_pt = cMathUtil::QuatRotVec(parent_to_parent_body, joint_attach_pt);
		joint_attach_pt -= parent_body_attach_pt;
		body_attach_pt = cMathUtil::QuatRotVec(body_to_this.inverse(), body_attach_pt);

		btTransform parent_body_to_body_trans = btTransform::getIdentity();
		parent_body_to_body_trans.setRotation(btQuaternion(parent_body_to_body.x(), parent_body_to_body.y(), parent_body_to_body.z(), parent_body_to_body.w()));

		bool disable_parent_collision = true;
		if (is_root && !fixed_base)
		{
			mMultBody->setupFixed(j, static_cast<btScalar>(mass), inertia, parent_joint,
				parent_body_to_body_trans.getRotation(),
				world_scale * btVector3(joint_attach_pt[0], joint_attach_pt[1], joint_attach_pt[2]),
				world_scale * btVector3(body_attach_pt[0], body_attach_pt[1], body_attach_pt[2]),
				disable_parent_collision);
		}
		else
		{
			switch (joint_type)
			{
			case cKinTree::eJointTypeRevolute:
			{
				tVector axis = tVector(0, 0, 1, 0);
				axis = cMathUtil::QuatRotVec(this_to_body, axis);

				mMultBody->setupRevolute(j, static_cast<btScalar>(mass), inertia, parent_joint,
					parent_body_to_body_trans.getRotation(),
					btVector3(axis[0], axis[1], axis[2]),
					world_scale * btVector3(joint_attach_pt[0], joint_attach_pt[1], joint_attach_pt[2]),
					world_scale * btVector3(body_attach_pt[0], body_attach_pt[1], body_attach_pt[2]),
					disable_parent_collision);
			}
			break;
			case cKinTree::eJointTypePlanar:
			{
				tVector offset = parent_body_attach_pt + joint_attach_pt
								+ cMathUtil::QuatRotVec(body_to_parent_body, body_attach_pt);
				
				tVector axis = tVector(0, 0, 1, 0);
				axis = cMathUtil::QuatRotVec(this_to_body, axis);

				mMultBody->setupPlanar(j, static_cast<btScalar>(mass), inertia, parent_joint,
					parent_body_to_body_trans.getRotation(),
					btVector3(axis[0], axis[1], axis[2]),
					world_scale * btVector3(offset[0], offset[1], offset[2]),
					disable_parent_collision);
			}
			break;
			case cKinTree::eJointTypePrismatic:
			{
				tVector axis = tVector(1, 0, 0, 0);
				axis = cMathUtil::QuatRotVec(this_to_body, axis);
				
				mMultBody->setupPrismatic(j, static_cast<btScalar>(mass), inertia, parent_joint,
					parent_body_to_body_trans.getRotation(),
					btVector3(axis[0], axis[1], axis[2]),
					world_scale * btVector3(joint_attach_pt[0], joint_attach_pt[1], joint_attach_pt[2]),
					world_scale * btVector3(body_attach_pt[0], body_attach_pt[1], body_attach_pt[2]),
					disable_parent_collision);
			}
			break;
			case cKinTree::eJointTypeFixed:
			{
				mMultBody->setupFixed(j, static_cast<btScalar>(mass), inertia, parent_joint,
					parent_body_to_body_trans.getRotation(),
					world_scale * btVector3(joint_attach_pt[0], joint_attach_pt[1], joint_attach_pt[2]),
					world_scale * btVector3(body_attach_pt[0], body_attach_pt[1], body_attach_pt[2]),
					disable_parent_collision);
			}
			break;
			case cKinTree::eJointTypeSpherical:
			{
				mMultBody->setupSpherical(j, static_cast<btScalar>(mass), inertia, parent_joint,
					parent_body_to_body_trans.getRotation(),
					world_scale * btVector3(joint_attach_pt[0], joint_attach_pt[1], joint_attach_pt[2]),
					world_scale * btVector3(body_attach_pt[0], body_attach_pt[1], body_attach_pt[2]),
					disable_parent_collision);
			}
			break;
			default:
				printf("Unsupported joint type");
				assert(false);
				break;
			}
		}

		btMultiBodyLinkCollider* col_obj = new btMultiBodyLinkCollider(mMultBody.get(), j);

		col_obj->setCollisionShape(col_shape);
		btTransform col_obj_trans;
		col_obj_trans.setIdentity();
		col_obj->setWorldTransform(col_obj_trans);
		col_obj->setFriction(mFriction);

		int collisionFilterGroup = GetPartColGroup(j);
		int collisionFilterMask = GetPartColMask(j);
		mWorld->AddCollisionObject(col_obj, collisionFilterGroup, collisionFilterMask);
		mMultBody->getLink(j).m_collider = col_obj;
	}

	mMultBody->finalizeMultiDof();

	return succ;
}

bool cSimCharacter::BuildConstraints(std::shared_ptr<cMultiBody>& out_body)
{
	double world_scale = mWorld->GetScale();
	for (int j = 0; j < GetNumJoints(); ++j)
	{
		cKinTree::eJointType joint_type = cKinTree::GetJointType(mJointMat, j);
		if (joint_type == cKinTree::eJointTypeRevolute || joint_type == cKinTree::eJointTypePrismatic)
		{
			tVector lim_low = cKinTree::GetJointLimLow(mJointMat, j);
			tVector lim_high = cKinTree::GetJointLimHigh(mJointMat, j);
			if (lim_low[0] <= lim_high[1])
			{
				if (joint_type == cKinTree::eJointTypePrismatic)
				{
					lim_low *= world_scale;
					lim_high *= world_scale;
				}

				auto joint_cons = std::shared_ptr<btMultiBodyJointLimitConstraint>(new btMultiBodyJointLimitConstraint(mMultBody.get(), j, lim_low[0], lim_high[0]));
				joint_cons->finalizeMultiDof();
				mCons.push_back(joint_cons);
			}
		}
	}
	return true;
}

bool cSimCharacter::LoadBodyDefs(const std::string& char_file, Eigen::MatrixXd& out_body_defs) const
{
	bool succ = cKinTree::LoadBodyDefs(char_file, out_body_defs);
	int num_joints = GetNumJoints();
	int num_body_defs = static_cast<int>(out_body_defs.rows());
	assert(num_joints == num_body_defs);
	return succ;
}

bool cSimCharacter::BuildBodyLinks()
{
	int num_joints = GetNumJoints();
	mBodyParts.clear();
	mBodyParts.resize(num_joints);

	for (int j = 0; j < num_joints; ++j)
	{
		std::shared_ptr<cSimBodyLink>& curr_part = mBodyParts[j];

		cSimBodyLink::tParams params;
		params.mMass = cKinTree::GetBodyMass(mBodyDefs, j);
		params.mJointID = j;

		curr_part = std::shared_ptr<cSimBodyLink>(new cSimBodyLink());

		short col_group = GetPartColGroup(j);
		short col_mask = GetPartColMask(j);
		curr_part->SetColGroup(col_group);
		curr_part->SetColMask(col_mask);

		curr_part->Init(mWorld, mMultBody, params);
	}

	return true;
}

btCollisionShape* cSimCharacter::BuildCollisionShape(const cShape::eShape shape, const tVector& shape_size)
{
	btCollisionShape* col_shape = nullptr;
	switch (shape)
	{
	case cShape::eShapeBox:
		col_shape = mWorld->BuildBoxShape(shape_size);
		break;
	case cShape::eShapeCapsule:
		col_shape = mWorld->BuildCapsuleShape(0.5 * shape_size[0], shape_size[1]);
		break;
	case cShape::eShapeSphere:
		col_shape = mWorld->BuildSphereShape(0.5 * shape_size[0]);
		break;
	case cShape::eShapeCylinder:
		col_shape = mWorld->BuildCylinderShape(0.5 * shape_size[0], shape_size[1]);
		break;
	default:
		printf("Unsupported body link shape\n");
		assert(false);
		break;
	}
	return col_shape;
}

bool cSimCharacter::BuildJoints()
{
	int num_joints = GetNumJoints();
	mJoints.clear();
	mJoints.resize(num_joints);
	
	for (int j = 0; j < num_joints; ++j)
	{
		bool is_root = cKinTree::IsRoot(mJointMat, j);
		cSimBodyJoint& curr_joint = GetJoint(j);

		cSimBodyJoint::tParams joint_params;
		joint_params.mID = j;
		joint_params.mLimLow = cKinTree::GetJointLimLow(mJointMat, j);
		joint_params.mLimHigh = cKinTree::GetJointLimHigh(mJointMat, j);
		joint_params.mTorqueLimit = (is_root) ? 0 : cKinTree::GetTorqueLimit(mJointMat, j);
		joint_params.mForceLimit = (is_root) ? 0 : cKinTree::GetForceLimit(mJointMat, j);

		tVector child_attach_pt = cKinTree::GetBodyAttachPt(mBodyDefs, j);
		tVector child_attach_theta = cKinTree::GetBodyAttachTheta(mBodyDefs, j);
		tMatrix child_to_joint = cMathUtil::TranslateMat(child_attach_pt) * cMathUtil::RotateMat(child_attach_theta);
		tMatrix joint_to_child = cMathUtil::InvRigidMat(child_to_joint);

		joint_params.mChildRot = cMathUtil::RotMatToQuaternion(joint_to_child);
		joint_params.mChildPos = cMathUtil::GetRigidTrans(joint_to_child);
		
		std::shared_ptr<cSimBodyLink> child_link = GetBodyPart(j);
		std::shared_ptr<cSimBodyLink> parent_link = nullptr;

		int parent_id = cKinTree::GetParent(mJointMat, j);
		if (parent_id != gInvalidIdx)
		{
			tVector joint_attach_pt = cKinTree::GetAttachPt(mJointMat, j);
			tVector joint_attach_theta = cKinTree::GetAttachTheta(mJointMat, j);
			tVector parent_attach_pt = cKinTree::GetBodyAttachPt(mBodyDefs, parent_id);
			tVector parent_attach_theta = cKinTree::GetBodyAttachTheta(mBodyDefs, parent_id);
			
			tMatrix parent_to_parent_joint = cMathUtil::TranslateMat(parent_attach_pt) * cMathUtil::RotateMat(parent_attach_theta);
			tMatrix joint_to_parent_joint = cMathUtil::TranslateMat(joint_attach_pt) * cMathUtil::RotateMat(joint_attach_theta);
			tMatrix joint_to_parent = cMathUtil::InvRigidMat(parent_to_parent_joint) * joint_to_parent_joint;

			parent_link = GetBodyPart(parent_id);
			joint_params.mParentRot = cMathUtil::RotMatToQuaternion(joint_to_parent);
			joint_params.mParentPos = cMathUtil::GetRigidTrans(joint_to_parent);
		}

		curr_joint.Init(mWorld, mMultBody, parent_link, child_link, joint_params);
	}

	return true;
}

void cSimCharacter::BuildConsFactor(int joint_id, tVector& out_linear_factor, tVector& out_angular_factor) const
{
	cKinTree::eJointType joint_type = cKinTree::GetJointType(mJointMat, joint_id);
	bool is_root = cKinTree::IsRoot(mJointMat, joint_id);
	out_linear_factor.setOnes();
	out_angular_factor.setOnes();

	if (is_root)
	{
		BuildRootConsFactor(joint_type, out_linear_factor, out_angular_factor);
	}
}

void cSimCharacter::BuildRootConsFactor(cKinTree::eJointType joint_type, tVector& out_linear_factor, tVector& out_angular_factor) const
{
	out_linear_factor = tVector::Ones();
	out_angular_factor = tVector::Ones();

	switch (joint_type)
	{
	case cKinTree::eJointTypeRevolute:
		out_linear_factor = tVector::Zero();
		out_angular_factor = tVector(0, 0, 1, 0);
		break;
	case cKinTree::eJointTypePlanar:
		out_linear_factor = tVector(1, 1, 0, 0);
		out_angular_factor = tVector(0, 0, 1, 0);
		break;
	case cKinTree::eJointTypePrismatic:
		out_linear_factor = tVector(0, 0, 1, 0);
		out_angular_factor = tVector::Zero();
		break;
	case cKinTree::eJointTypeFixed:
		out_linear_factor = tVector::Zero();
		out_angular_factor = tVector::Zero();
		break;
	case cKinTree::eJointTypeNone:
		out_linear_factor = tVector::Ones();
		out_angular_factor = tVector::Ones();
		break;
	case cKinTree::eJointTypeSpherical:
		out_linear_factor = tVector::Zero();
		out_angular_factor = tVector(1, 1, 1, 0);
		break;
	default:
		assert(false); // unsupported joint type
		break;
	}
}

bool cSimCharacter::FixedBase() const
{
	int root_id = GetRootID();
	cKinTree::eJointType joint_type = cKinTree::GetJointType(mJointMat, root_id);
	return joint_type != cKinTree::eJointTypeNone;
}

void cSimCharacter::RemoveFromWorld()
{
	if (mWorld != nullptr)
	{
		mWorld->RemoveCharacter(*this);
		mJoints.clear();
		mBodyParts.clear();
		mCons.clear();
		mMultBody.reset();
		mWorld.reset();
	}
}

bool cSimCharacter::IsValidBodyPart(int idx) const
{
	return mBodyParts[idx] != nullptr;
}

bool cSimCharacter::EnableBodyPartFallContact(int idx) const
{
	assert(idx >= 0 && idx < GetNumBodyParts());
	return cKinTree::GetBodyEnableFallContact(mBodyDefs, idx);
}

void cSimCharacter::SetBodyPartFallContact(int idx, bool enable)
{
	assert(idx >= 0 && idx < GetNumBodyParts());
	cKinTree::SetBodyEnableFallContact(idx, enable, mBodyDefs);
}

tMatrix cSimCharacter::BuildJointWorldTrans(int joint_id) const
{
	const cSimBodyJoint& joint = GetJoint(joint_id);
	if (joint.IsValid())
	{
		return joint.BuildWorldTrans();
	}
	else
	{
		return cCharacter::BuildJointWorldTrans(joint_id);
	}
}

void cSimCharacter::ClearJointTorques()
{
	int num_joints = GetNumJoints();
	for (int j = 1; j < num_joints; ++j)
	{
		cSimBodyJoint& joint = GetJoint(j);
		if (joint.IsValid())
		{
			joint.ClearTau();
		}
	}
}

void cSimCharacter::UpdateJoints()
{
	int num_joints = GetNumJoints();
	for (int j = 1; j < num_joints; ++j)
	{
		cSimBodyJoint& joint = GetJoint(j);
		if (joint.IsValid())
		{
			joint.ApplyTau();
		}
	}
}

void cSimCharacter::UpdateLinkPos()
{
	mMultBody->updateCollisionObjectWorldTransforms(mRotBuffer, mVecBuffer0);
}

void cSimCharacter::UpdateLinkVel()
{
	static double max_vel_err = 0;
	static double max_omega_err = 0;
	
	const auto& joint_mat = GetJointMat();
	const auto& body_defs = GetBodyDefs();
	const auto& pose = GetPose();
	const auto& vel = GetVel();

	btAlignedObjectArray<btVector3>& ang_vel_buffer = mVecBuffer0;
	btAlignedObjectArray<btVector3>& vel_buffer = mVecBuffer1;

	mMultBody->compTreeLinkVelocities(&ang_vel_buffer[0], &vel_buffer[0]);

	double world_scale = mWorld->GetScale();
	for (int b = 0; b < GetNumBodyParts(); ++b)
	{
		if (IsValidBodyPart(b))
		{
			const btVector3& bt_vel = vel_buffer[b + 1];
			const btVector3& bt_ang_vel = ang_vel_buffer[b + 1];

			tVector com_vel = tVector(bt_vel[0], bt_vel[1], bt_vel[2], 0);
			tVector com_omega = tVector(bt_ang_vel[0], bt_ang_vel[1], bt_ang_vel[2], 0);
			com_vel /= world_scale;

			// velocities are in the body's local coordinates
			// so need to transform them into world coords
			tQuaternion world_rot = GetBodyPart(b)->GetRotation();
			com_vel = cMathUtil::QuatRotVec(world_rot, com_vel);
			com_omega = cMathUtil::QuatRotVec(world_rot, com_omega);

			const auto& link = GetBodyPart(b);
			link->UpdateVel(com_vel, com_omega);
		}
	}
}

short cSimCharacter::GetPartColGroup(int part_id) const
{
	return GetPartColMask(part_id);
}

short cSimCharacter::GetPartColMask(int part_id) const
{
	int col_group = cKinTree::GetBodyColGroup(mBodyDefs, part_id);
	assert(col_group < static_cast<int>(sizeof(short) * 8));

	short flags;
	if (col_group == 0)
	{
		flags = cContactManager::gFlagNone;
	}
	else if (col_group == -1)
	{
		flags = cContactManager::gFlagAll;
	}
	else
	{
		flags = 1 << col_group;
	}
	return flags;
}

tVector cSimCharacter::GetPartColor(int part_id) const
{
	return cKinTree::GetBodyColor(mBodyDefs, part_id);
}

double cSimCharacter::CalcTotalMass() const
{
	double m = 0;
	for (int i = 0; i < GetNumBodyParts(); ++i)
	{
		if (IsValidBodyPart(i))
		{
			m += GetBodyPart(i)->GetMass();
		}
	}
	return m;
}

void cSimCharacter::SetLinearDamping(double damping)
{
	mMultBody->setLinearDamping(damping);
}

void cSimCharacter::SetAngularDamping(double damping)
{
	mMultBody->setAngularDamping(damping);
}

tVector cSimCharacter::GetPos() const
{
	return GetRootPos();
}

void cSimCharacter::SetPos(const tVector& pos)
{
	SetRootPos(pos);
}

void cSimCharacter::GetRotation(tVector& out_axis, double& out_theta) const
{
	return GetRootRotation(out_axis, out_theta);
}

tQuaternion cSimCharacter::GetRotation() const
{
	return GetRootRotation();
}

void cSimCharacter::SetRotation(const tVector& axis, double theta)
{
	SetRootRotation(axis, theta);
}

void cSimCharacter::SetRotation(const tQuaternion& q)
{
	SetRootRotation(q);
}

tMatrix cSimCharacter::GetWorldTransform() const
{
	return GetJoint(GetRootID()).BuildWorldTrans();
}

tVector cSimCharacter::GetLinearVelocity() const
{
	return GetRootVel();
}

tVector cSimCharacter::GetLinearVelocity(const tVector& local_pos) const
{
	const auto& root_body = GetRootPart();
	const auto& joint = GetJoint(GetRootID());

	tVector world_pos = local_pos;
	world_pos[3] = 1;
	tMatrix joint_to_world = joint.BuildWorldTrans();
	world_pos = joint_to_world * world_pos;

	tVector body_local = root_body->WorldToLocalPos(world_pos);
	return root_body->GetLinearVelocity(body_local);
}

void cSimCharacter::SetLinearVelocity(const tVector& vel)
{
	SetRootVel(vel);
}

tVector cSimCharacter::GetAngularVelocity() const
{
	return GetRootAngVel();
}

void cSimCharacter::SetAngularVelocity(const tVector& vel)
{
	SetRootAngVel(vel);
}

short cSimCharacter::GetColGroup() const
{
	return GetRootPart()->GetColGroup();
}

void cSimCharacter::SetColGroup(short col_group)
{
	for (int i = 0; i < GetNumBodyParts(); ++i)
	{
		if (IsValidBodyPart(i))
		{
			GetBodyPart(i)->SetColGroup(col_group);
		}
	}
}

short cSimCharacter::GetColMask() const
{
	return GetRootPart()->GetColMask();
}

void cSimCharacter::SetColMask(short col_mask)
{
	for (int i = 0; i < GetNumBodyParts(); ++i)
	{
		if (IsValidBodyPart(i))
		{
			GetBodyPart(i)->SetColMask(col_mask);
		}
	}
}

const std::shared_ptr<cWorld>& cSimCharacter::GetWorld() const
{
	return mWorld;
}

const std::shared_ptr<cMultiBody>& cSimCharacter::GetMultiBody() const
{
	return mMultBody;
}

const std::vector<std::shared_ptr<btMultiBodyJointLimitConstraint>>& cSimCharacter::GetConstraints() const
{
	return mCons;
}

void cSimCharacter::BuildJointPose(int joint_id, Eigen::VectorXd& out_pose) const
{
	bool is_root = cKinTree::IsRoot(mJointMat, joint_id);
	if (is_root)
	{
		int param_size = GetParamSize(joint_id);
		out_pose = Eigen::VectorXd::Zero(param_size);
		assert(out_pose.size() == cKinTree::gRootDim);

		tVector root_pos = GetRootPos();
		tQuaternion root_rot = GetRootRotation();

		out_pose.segment(0, cKinTree::gPosDim) = root_pos.segment(0, cKinTree::gPosDim);
		out_pose(cKinTree::gPosDim) = root_rot.w();
		out_pose(cKinTree::gPosDim + 1) = root_rot.x();
		out_pose(cKinTree::gPosDim + 2) = root_rot.y();
		out_pose(cKinTree::gPosDim + 3) = root_rot.z();
	}
	else
	{
		const cSimBodyJoint& joint = GetJoint(joint_id);
		joint.BuildPose(out_pose);
	}
}

void cSimCharacter::BuildJointVel(int joint_id, Eigen::VectorXd& out_vel) const
{
	bool is_root = cKinTree::IsRoot(mJointMat, joint_id);
	if (is_root)
	{
		int param_size = GetParamSize(joint_id);
		out_vel = Eigen::VectorXd::Zero(param_size);
		assert(out_vel.size() == cKinTree::gRootDim);

		tVector root_vel = GetRootVel();
		tVector ang_vel = GetRootAngVel();
		out_vel.segment(0, cKinTree::gPosDim) = root_vel.segment(0, cKinTree::gPosDim);
		out_vel.segment(cKinTree::gPosDim, cKinTree::gRotDim) = ang_vel.segment(0, cKinTree::gRotDim);
	}
	else
	{
		const cSimBodyJoint& joint = GetJoint(joint_id);
		joint.BuildVel(out_vel);
	}
}

void cSimCharacter::BuildPose(Eigen::VectorXd& out_pose) const
{
	int num_joints = GetNumJoints();
	int num_dof = cKinTree::GetNumDof(mJointMat);
	out_pose.resize(num_dof);
	for (int j = 0; j < num_joints; ++j)
	{
		Eigen::VectorXd joint_pose;
		BuildJointPose(j, joint_pose);

		int param_offset = GetParamOffset(j);
		int param_size = GetParamSize(j);
		assert(joint_pose.size() == param_size);
		out_pose.segment(param_offset, param_size) = joint_pose;
	}
}

void cSimCharacter::BuildVel(Eigen::VectorXd& out_vel) const
{
	int num_joints = GetNumJoints();
	int num_dof = cKinTree::GetNumDof(mJointMat);
	out_vel.resize(num_dof);

	for (int j = 0; j < num_joints; ++j)
	{
		Eigen::VectorXd joint_vel;
		BuildJointVel(j, joint_vel);

		int param_offset = GetParamOffset(j);
		int param_size = GetParamSize(j);
		assert(joint_vel.size() == param_size);
		out_vel.segment(param_offset, param_size) = joint_vel;
	}
}

bool cSimCharacter::CheckFallContact() const
{
	int num_parts = GetNumBodyParts();
	for (int b = 0; b < num_parts; ++b)
	{
		if (IsValidBodyPart(b) && EnableBodyPartFallContact(b))
		{
			const auto& curr_part = GetBodyPart(b);
			bool has_contact = curr_part->IsInContact();
			if (has_contact)
			{
				return true;
			}
		}
	}
	return false;
}

const btCollisionObject* cSimCharacter::GetCollisionObject() const
{
	return nullptr;
}

btCollisionObject* cSimCharacter::GetCollisionObject()
{
	return nullptr;
}