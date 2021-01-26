#include <iostream>
#include "SimBodyJoint.h"
#include "BulletDynamics/Featherstone/btMultiBodyLink.h"

cSimBodyJoint::tParams::tParams()
{
	mID = gInvalidIdx;
	mLimLow.setOnes(); // low > high -> no limit
	mLimHigh.setZero();
	mTorqueLimit = std::numeric_limits<double>::infinity();
	mForceLimit = std::numeric_limits<double>::infinity();

	mParentPos.setZero();
	mChildPos.setZero();
	mParentRot.setIdentity();
	mChildRot.setIdentity();
}


cSimBodyJoint::cSimBodyJoint()
{
}

cSimBodyJoint::~cSimBodyJoint()
{
	Clear();
}

void cSimBodyJoint::Init(std::shared_ptr<cWorld>& world, const std::shared_ptr<cMultiBody>& multbody,
						const std::shared_ptr<cSimBodyLink>& parent, const std::shared_ptr<cSimBodyLink>& child, 
						const tParams& params)
{
	Clear();
	mWorld = world;
	mMultBody = multbody;
	mParent = parent;
	mChild = child;
	mParams = params;
	mType = FetchJointType();

	if (!HasChild())
	{
		assert(mType == cKinTree::eJointTypeFixed);
	}

	assert(IsValid());
}

void cSimBodyJoint::Clear()
{
	mType = cKinTree::eJointTypeNone;
	mParams = tParams();
	mWorld.reset();
	mParent.reset();
	mChild.reset();
	ClearTau();
}

bool cSimBodyJoint::IsValid() const
{
	return(mWorld != nullptr) && (mChild != nullptr || mParent != nullptr);
}

tVector cSimBodyJoint::CalcAxisWorld() const
{
	tVector axis_rel = GetAxisRel();
	axis_rel[3] = 0;
	tMatrix trans = BuildWorldTrans();
	tVector axis = trans * axis_rel;
	return axis;
}

tVector cSimBodyJoint::GetAxisRel() const
{
	return tVector(0, 0, 1, 0);
}

bool cSimBodyJoint::HasParent() const
{
	return mParent != nullptr;
}

bool cSimBodyJoint::HasChild() const
{
	return mChild != nullptr;
}

cKinTree::eJointType cSimBodyJoint::GetType() const
{
	return mType;
}

tQuaternion cSimBodyJoint::CalcWorldRotation() const
{
	tMatrix mat = BuildWorldTrans();
	tQuaternion q = cMathUtil::RotMatToQuaternion(mat);
	return q;
}

void cSimBodyJoint::CalcWorldRotation(tVector& out_axis, double& out_theta) const
{
	tMatrix mat = BuildWorldTrans();
	cMathUtil::RotMatToAxisAngle(mat, out_axis, out_theta);
}

tMatrix cSimBodyJoint::BuildWorldTrans() const
{
	tMatrix mat = tMatrix::Identity();
	if (HasChild())
	{
		mat = mChild->GetWorldTransform();
		mat = mat * BuildJointChildTrans();
	}
	else
	{
		mat = mParent->GetWorldTransform();
		mat = mat * BuildJointParentTrans();
	}
	return mat;
}

bool cSimBodyJoint::IsRoot() const
{
	return mParams.mID == 0;
}


void cSimBodyJoint::AddTau(const Eigen::VectorXd& tau)
{
	switch (GetType())
	{
	case cKinTree::eJointTypeRevolute:
		mTotalTau[2] += tau[0];
		break;
	case cKinTree::eJointTypePlanar:
		mTotalTau[3] += tau[0];
		mTotalTau[4] += tau[1];
		mTotalTau[2] += tau[2];
		break;
	case cKinTree::eJointTypePrismatic:
		mTotalTau[3] += tau[0];
		break;
	case cKinTree::eJointTypeFixed:
		break;
	case cKinTree::eJointTypeSpherical:
		mTotalTau[0] += tau[0];
		mTotalTau[1] += tau[1];
		mTotalTau[2] += tau[2];
		break;
	default:
		assert(false); // unsupported joint type
		break;
	}
}

const cSpAlg::tSpVec& cSimBodyJoint::GetTau() const
{
	return mTotalTau;
}

void cSimBodyJoint::ApplyTau()
{
	switch (GetType())
	{
	case cKinTree::eJointTypeRevolute:
		ApplyTauRevolute();
		break;
	case cKinTree::eJointTypePlanar:
		ApplyTauPlanar();
		break;
	case cKinTree::eJointTypePrismatic:
		ApplyTauPrismatic();
		break;
	case cKinTree::eJointTypeFixed:
		break;
	case cKinTree::eJointTypeSpherical:
		ApplyTauSpherical();
		break;
	default:
		assert(false); // unsupported joint type
		break;
	}
}

void cSimBodyJoint::ClearTau()
{
	mTotalTau.setZero();
}

tVector cSimBodyJoint::CalcWorldPos() const
{
	tMatrix trans = BuildWorldTrans();
	tVector world_pos = tVector(trans(0, 3), trans(1, 3), trans(2, 3), 0);
	return world_pos;
}

tVector cSimBodyJoint::CalcWorldPos(const tVector& local_pos) const
{
	tVector world_pos = local_pos;
	world_pos[3] = 1;
	tMatrix trans = BuildWorldTrans();
	world_pos = trans * world_pos;
	world_pos[3] = 0;

	return world_pos;
}

tVector cSimBodyJoint::CalcWorldVel() const
{
	if (HasChild())
	{
		tVector anchor = GetChildPos();
		return mChild->GetLinearVelocity(anchor);
	}
	else
	{
		tVector anchor = GetParentPos();
		return mParent->GetLinearVelocity(anchor);
	}
}

tVector cSimBodyJoint::CalcWorldVel(const tVector& local_pos) const
{
	if (HasChild())
	{
		tVector pt = CalcChildLocalPos(local_pos);
		return mChild->GetLinearVelocity(pt);
	}
	else
	{
		
		tVector pt = CalcParentLocalPos(local_pos);
		return mParent->GetLinearVelocity(pt);
	}
}

tVector cSimBodyJoint::CalcWorldAngVel() const
{
	if (HasChild())
	{
		return mChild->GetAngularVelocity();
	}
	else
	{
		return mParent->GetAngularVelocity();
	}
}

double cSimBodyJoint::GetTorqueLimit() const
{
	return mParams.mTorqueLimit;
}

double cSimBodyJoint::GetForceLimit() const
{
	return mParams.mForceLimit;
}

void cSimBodyJoint::SetTorqueLimit(double lim)
{
	mParams.mTorqueLimit = lim;
}

void cSimBodyJoint::SetForceLimit(double lim)
{
	mParams.mForceLimit = lim;
}

tVector cSimBodyJoint::GetParentPos() const
{
	return mParams.mParentPos;
}

tVector cSimBodyJoint::GetChildPos() const
{
	return mParams.mChildPos; 
}

tMatrix cSimBodyJoint::BuildJointChildTrans() const
{
	return cMathUtil::TranslateMat(mParams.mChildPos) * cMathUtil::RotateMat(mParams.mChildRot);
}

tMatrix cSimBodyJoint::BuildJointParentTrans() const
{
	return cMathUtil::TranslateMat(mParams.mParentPos) * cMathUtil::RotateMat(mParams.mParentRot);
}

const std::shared_ptr<cSimBodyLink>& cSimBodyJoint::GetParent() const
{
	return mParent;
}

const std::shared_ptr<cSimBodyLink>& cSimBodyJoint::GetChild() const
{
	return mChild;
}

void cSimBodyJoint::ClampTotalTorque(tVector& out_torque) const
{
	double mag = out_torque.norm();
	double torque_lim = GetTorqueLimit();
	if (mag > torque_lim)
	{
		out_torque *= torque_lim / mag;
	}
}

void cSimBodyJoint::ClampTotalForce(tVector& out_force) const
{
	double mag = out_force.norm();
	double force_lim = GetForceLimit();
	if (mag > force_lim)
	{
		out_force *= force_lim / mag;
	}
}

const tVector& cSimBodyJoint::GetLimLow() const
{
	return mParams.mLimLow;
}

const tVector& cSimBodyJoint::GetLimHigh() const
{
	return mParams.mLimHigh;
}

bool cSimBodyJoint::HasJointLim() const
{
	return (mParams.mLimLow[0] <= mParams.mLimHigh[0]
			|| mParams.mLimLow[1] <= mParams.mLimHigh[1]
			|| mParams.mLimLow[2] <= mParams.mLimHigh[2]
			|| mParams.mLimLow[3] <= mParams.mLimHigh[3]);
}

int cSimBodyJoint::GetParamSize() const
{
	return cKinTree::GetJointParamSize(GetType());
}

void cSimBodyJoint::BuildPose(Eigen::VectorXd& out_pose) const
{
	const btScalar* data = mMultBody->getJointPosMultiDof(mParams.mID);
	double world_scale = mWorld->GetScale();
	int param_size = GetParamSize();
	out_pose.resize(param_size);

	switch (GetType())
	{
	case cKinTree::eJointTypeRevolute:
		out_pose[0] = cMathUtil::NormalizeAngle(data[0]);
		break;
	case cKinTree::eJointTypePlanar:
	{
		const auto& link = mMultBody->getLink(mParams.mID);
		btVector3 bt_axis1 = link.getAxisBottom(1);
		btVector3 bt_axis2 = link.getAxisBottom(2);
		tVector axis1 = tVector(bt_axis1[0], bt_axis1[1], bt_axis1[2], 0);
		tVector axis2 = tVector(bt_axis2[0], bt_axis2[1], bt_axis2[2], 0);
		tVector pos = axis2 * (data[2] / world_scale) + axis1 * (data[1] / world_scale);
		pos = cMathUtil::QuatRotVec(mParams.mChildRot.conjugate(), pos);

		out_pose[0] = pos[0];
		out_pose[1] = pos[1];
		out_pose[2] = cMathUtil::NormalizeAngle(data[0]);
		break;
	}
	case cKinTree::eJointTypePrismatic:
		out_pose[0] = data[0] / world_scale;
		break;
	case cKinTree::eJointTypeFixed:
		break;
	case cKinTree::eJointTypeSpherical:
	{
		tQuaternion q = tQuaternion(data[3], data[0], data[1], data[2]);
		q = mParams.mChildRot.conjugate() * q * mParams.mChildRot;
		bool flip = (q.w() < 0);
		out_pose[0] = (flip) ? -q.w() : q.w();
		out_pose[1] = (flip) ? -q.x() : q.x();
		out_pose[2] = (flip) ? -q.y() : q.y();
		out_pose[3] = (flip) ? -q.z() : q.z();
		break;
	}
	default:
		assert(false); // unsupported joint type
		break;
	}
}

void cSimBodyJoint::BuildVel(Eigen::VectorXd& out_vel) const
{
	const btScalar* data = mMultBody->getJointVelMultiDof(mParams.mID);
	double world_scale = mWorld->GetScale();
	int param_size = GetParamSize();
	out_vel.resize(param_size);

	switch (GetType())
	{
	case cKinTree::eJointTypeRevolute:
		out_vel[0] = data[0];
		break;
	case cKinTree::eJointTypePlanar:
	{
		const auto& link = mMultBody->getLink(mParams.mID);
		tMatrix body_to_joint = cMathUtil::InvRigidMat(BuildJointChildTrans());
		const btVector3& axis0 = link.getAxisTop(0);
		const btVector3& axis1 = link.getAxisBottom(1);
		const btVector3& axis2 = link.getAxisBottom(2);

		tVector lin_vel = (data[2] / world_scale) * tVector(axis2[0], axis2[1], axis2[2], 0)
						+ (data[1] / world_scale) * tVector(axis1[0], axis1[1], axis1[2], 0);
		tVector ang_vel = data[0] * tVector(axis0[0], axis0[1], axis0[2], 0);

		cSpAlg::tSpTrans trans = cSpAlg::MatToTrans(body_to_joint);
		cSpAlg::tSpVec sv = cSpAlg::BuildSV(ang_vel, lin_vel);
		sv = cSpAlg::ApplyTransM(trans, sv);
		lin_vel = cSpAlg::GetV(sv);
		ang_vel = cSpAlg::GetOmega(sv);

		out_vel[0] = lin_vel[0];
		out_vel[1] = lin_vel[1];
		out_vel[2] = ang_vel[2];
		break;
	}
	case cKinTree::eJointTypePrismatic:
		out_vel[0] = data[0] / world_scale;
		break;
	case cKinTree::eJointTypeFixed:
		break;
	case cKinTree::eJointTypeSpherical:
	{
		tVector ang_vel = tVector(data[0], data[1], data[2], 0);
		ang_vel = cMathUtil::QuatRotVec(mParams.mChildRot.conjugate(), ang_vel);
		out_vel[0] = ang_vel[0];
		out_vel[1] = ang_vel[1];
		out_vel[2] = ang_vel[2];
		out_vel[3] = 0;
		break;
	}
	default:
		assert(false); // unsupported joint type
		break;
	}
}

void cSimBodyJoint::SetPose(const Eigen::VectorXd& pose)
{
	btScalar data[7];
	double world_scale = mWorld->GetScale();

	switch (GetType())
	{
	case cKinTree::eJointTypeRevolute:
		data[0] = pose[0];
		break;
	case cKinTree::eJointTypePlanar:
	{
		tVector pos = cMathUtil::QuatRotVec(mParams.mChildRot, tVector(pose[0], pose[1], 0, 0));
		const auto& link = mMultBody->getLink(mParams.mID);

		data[2] = link.getAxisBottom(2).dot(btVector3(pos[0], pos[1], pos[2])) * world_scale;
		data[1] = link.getAxisBottom(1).dot(btVector3(pos[0], pos[1], pos[2])) * world_scale;
		data[0] = pose[2];
		break;
	}
	case cKinTree::eJointTypePrismatic:
		data[0] = pose[0] * world_scale;
		break;
	case cKinTree::eJointTypeFixed:
		break;
	case cKinTree::eJointTypeSpherical:
	{
		tQuaternion q = cMathUtil::VecToQuat(pose);
		q = mParams.mChildRot * q * mParams.mChildRot.conjugate();
		data[0] = q.x();
		data[1] = q.y();
		data[2] = q.z();
		data[3] = q.w();
		break;
	}
	case cKinTree::eJointTypeNone:
		break;
	default:
		assert(false); // unsupported joint type
		break;
	}

	mMultBody->setJointPosMultiDof(mParams.mID, data);
}

void cSimBodyJoint::SetVel(const Eigen::VectorXd& vel)
{
	btScalar data[7];
	double world_scale = mWorld->GetScale();

	switch (GetType())
	{
	case cKinTree::eJointTypeRevolute:
		data[0] = vel[0];
		break;
	case cKinTree::eJointTypePlanar:
	{
		tMatrix joint_to_body = BuildJointChildTrans();
		const auto& link = mMultBody->getLink(mParams.mID);

		tVector lin_vel = tVector(vel[0], vel[1], 0, 0);
		tVector ang_vel = tVector(0, 0, vel[2], 0);
		cSpAlg::tSpTrans trans = cSpAlg::MatToTrans(joint_to_body);
		cSpAlg::tSpVec sv = cSpAlg::BuildSV(ang_vel, lin_vel);
		sv = cSpAlg::ApplyTransM(trans, sv);
		lin_vel = cSpAlg::GetV(sv);

		data[2] = link.getAxisBottom(2).dot(btVector3(lin_vel[0], lin_vel[1], lin_vel[2])) * world_scale;
		data[1] = link.getAxisBottom(1).dot(btVector3(lin_vel[0], lin_vel[1], lin_vel[2])) * world_scale;
		data[0] = ang_vel[2];
		break;
	}
	case cKinTree::eJointTypePrismatic:
		data[0] = vel[0] * world_scale;
		break;
	case cKinTree::eJointTypeFixed:
		break;
	case cKinTree::eJointTypeSpherical:
	{
		tVector ang_vel = tVector(vel[0], vel[1], vel[2], 0);
		ang_vel = cMathUtil::QuatRotVec(mParams.mChildRot, ang_vel);
		data[0] = ang_vel[0];
		data[1] = ang_vel[1];
		data[2] = ang_vel[2];
		data[3] = 0;
		break;
	}
	case cKinTree::eJointTypeNone:
		break;
	default:
		assert(false); // unsupported joint type
		break;
	}

	mMultBody->setJointVelMultiDof(mParams.mID, data);
}

tVector cSimBodyJoint::GetTotalTorque() const
{
	tVector torque = cSpAlg::GetOmega(mTotalTau);
	return torque;
}

tVector cSimBodyJoint::CalcParentLocalPos(const tVector& local_pos) const
{
	cKinTree::eJointType joint_type = GetType();
	tVector pt = local_pos;
	double world_scale = mWorld->GetScale();
	const btScalar* data = mMultBody->getJointPosMultiDof(mParams.mID);
	if (joint_type == cKinTree::eJointTypePrismatic)
	{
		double delta = data[0] / world_scale;
		pt[0] -= delta;
	}
	else if (joint_type == cKinTree::eJointTypePlanar)
	{
		const auto& link = mMultBody->getLink(mParams.mID);
		btVector3 bt_axis1 = link.getAxisBottom(1);
		btVector3 bt_axis2 = link.getAxisBottom(2);
		tVector axis1 = tVector(bt_axis1[0], bt_axis1[1], bt_axis1[2], 0);
		tVector axis2 = tVector(bt_axis2[0], bt_axis2[1], bt_axis2[2], 0);
		tVector pos = axis2 * (data[2] / world_scale) + axis1 * (data[1] / world_scale);
		pos = cMathUtil::QuatRotVec(mParams.mChildRot.conjugate(), pos);

		pt[0] += pos[0];
		pt[1] += pos[1];
	}

	tVector parent_pos = cMathUtil::QuatRotVec(mParams.mParentRot, pt);
	parent_pos += mParams.mParentPos;
	return parent_pos;
}

tVector cSimBodyJoint::CalcChildLocalPos(const tVector& local_pos) const
{
	tVector child_pos = cMathUtil::QuatRotVec(mParams.mChildRot, local_pos);
	child_pos += mParams.mChildPos;
	return child_pos;
}

void cSimBodyJoint::SetTotalTorque(const tVector& torque)
{
	cSpAlg::SetOmega(torque, mTotalTau);
}

cKinTree::eJointType cSimBodyJoint::FetchJointType() const
{
	btMultibodyLink::eFeatherstoneJointType bt_type = mMultBody->getLink(mParams.mID).m_jointType;
	cKinTree::eJointType joint_type = cKinTree::eJointTypeNone;

	bool is_root = IsRoot();
	if (is_root && !mMultBody->hasFixedBase())
	{
		joint_type = cKinTree::eJointTypeNone;
	}
	else
	{
		switch (bt_type)
		{
		case btMultibodyLink::eRevolute:
			joint_type = cKinTree::eJointTypeRevolute;
			break;
		case btMultibodyLink::ePrismatic:
			joint_type = cKinTree::eJointTypePrismatic;
			break;
		case btMultibodyLink::eSpherical:
			joint_type = cKinTree::eJointTypeSpherical;
			break;
		case btMultibodyLink::ePlanar:
			joint_type = cKinTree::eJointTypePlanar;
			break;
		case btMultibodyLink::eFixed:
			joint_type = cKinTree::eJointTypeFixed;
			break;
		default:
			assert(false); // unsupported joint type
			break;
		}
	}

	return joint_type;
}

tVector cSimBodyJoint::GetTotalForce() const
{
	tVector force = cSpAlg::GetV(mTotalTau);
	return force;
}

void cSimBodyJoint::ApplyTauRevolute()
{
	tVector torque = GetTotalTorque();
	ClampTotalTorque(torque);
	SetTotalTorque(torque);

	double world_scale = mWorld->GetScale();
	btScalar bt_data[] = { static_cast<btScalar>(world_scale * world_scale * torque[2]) };
	mMultBody->addJointTorqueMultiDof(mParams.mID, bt_data);
}

void cSimBodyJoint::ApplyTauPlanar()
{
	tVector torque = GetTotalTorque();
	ClampTotalTorque(torque);
	SetTotalTorque(torque);

	tVector force = GetTotalForce();
	ClampTotalForce(force);
	SetTotalForce(force);

	const auto& link = mMultBody->getLink(mParams.mID);

	cSpAlg::tSpTrans trans = cSpAlg::MatToTrans(BuildJointChildTrans());
	cSpAlg::tSpVec sv = cSpAlg::BuildSV(torque, force);
	sv = cSpAlg::ApplyTransF(trans, sv);
	force = cSpAlg::GetV(sv);
	torque = cSpAlg::GetOmega(sv);

	double world_scale = mWorld->GetScale();
	btScalar bt_data[] = { static_cast<btScalar>(world_scale * world_scale * link.getAxisTop(0).dot(btVector3(torque[0], torque[1], torque[2]))),
							static_cast<btScalar>(world_scale * link.getAxisBottom(1).dot(btVector3(force[0], force[1], force[2]))),
							static_cast<btScalar>(world_scale * link.getAxisBottom(2).dot(btVector3(force[0], force[1], force[2]))) };
	mMultBody->addJointTorqueMultiDof(mParams.mID, bt_data);
}

void cSimBodyJoint::ApplyTauPrismatic()
{
	tVector force = GetTotalForce();
	ClampTotalForce(force);
	SetTotalForce(force);

	double world_scale = mWorld->GetScale();
	btScalar bt_data[] = { static_cast<btScalar>(world_scale * force[0]) };
	mMultBody->addJointTorqueMultiDof(mParams.mID, bt_data);
}

void cSimBodyJoint::ApplyTauSpherical()
{
	tVector torque = GetTotalTorque();
	ClampTotalTorque(torque);
	SetTotalTorque(torque);

	double world_scale = mWorld->GetScale();
	torque = cMathUtil::QuatRotVec(mParams.mChildRot, torque);
	btScalar bt_data[] = { static_cast<btScalar>(world_scale * world_scale * torque[0]),
							static_cast<btScalar>(world_scale * world_scale * torque[1]),
							static_cast<btScalar>(world_scale * world_scale * torque[2]) };
	mMultBody->addJointTorqueMultiDof(mParams.mID, bt_data);
}

void cSimBodyJoint::SetTotalForce(const tVector& force)
{
	cSpAlg::SetV(force, mTotalTau);
}
