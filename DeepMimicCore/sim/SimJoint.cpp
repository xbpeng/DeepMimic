#include <iostream>
#include "SimJoint.h"
#include "SimBodyLink.h"
#include "SimRigidBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyLink.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"

cSimJoint::tParams::tParams()
{
	mID = gInvalidIdx;
	mType = cKinTree::eJointTypeNone;
	mLimLow.setOnes(); // low > high -> no limit
	mLimHigh.setZero();
	mTorqueLimit = std::numeric_limits<double>::infinity();
	mForceLimit = std::numeric_limits<double>::infinity();
	mEnableAdjacentCollision = false;

	mParentPos.setZero();
	mChildPos.setZero();
	mParentRot.setIdentity();
	mChildRot.setIdentity();
}


cSimJoint::cSimJoint()
{
}

cSimJoint::~cSimJoint()
{
	RemoveFromWorld();
	mWorld.reset();
}

void cSimJoint::Init(std::shared_ptr<cWorld>& world, const std::shared_ptr<cSimObj>& parent, 
					const std::shared_ptr<cSimObj>& child, const tParams& params)
{
	RemoveFromWorld();
	mParent = parent;
	mChild = child;
	mParams = params;
	mWorld = world;

	BuildConstraint(mWorld);
	assert(IsValid());
}

bool cSimJoint::IsValid() const
{
	return mCons != nullptr || mMultBodyCons != nullptr;
}

tMatrix cSimJoint::BuildJointChildTrans() const
{
	return cMathUtil::TranslateMat(GetChildPos()) * cMathUtil::RotateMat(GetChildRot());
}

tMatrix cSimJoint::BuildJointParentTrans() const
{
	return cMathUtil::TranslateMat(GetParentPos()) * cMathUtil::RotateMat(GetParentRot());
}

tVector cSimJoint::CalcAxisWorld() const
{
	tVector axis_rel = GetAxisRel();
	axis_rel[3] = 0;
	tMatrix trans = BuildWorldTrans();
	tVector axis = trans * axis_rel;
	return axis;
}

tVector cSimJoint::GetAxisRel() const
{
	return tVector(0, 0, 1, 0);
}

bool cSimJoint::HasParent() const
{
	return mParent != nullptr;
}

bool cSimJoint::HasChild() const
{
	return mChild != nullptr;
}

int cSimJoint::GetID() const
{
	return mParams.mID;
}

void cSimJoint::SetID(int id)
{
	mParams.mID = id;
}

cKinTree::eJointType cSimJoint::GetType() const
{
	return mParams.mType;
}

tQuaternion cSimJoint::CalcWorldRotation() const
{
	tMatrix mat = BuildWorldTrans();
	tQuaternion q = cMathUtil::RotMatToQuaternion(mat);
	return q;
}

void cSimJoint::CalcWorldRotation(tVector& out_axis, double& out_theta) const
{
	tMatrix mat = BuildWorldTrans();
	cMathUtil::RotMatToAxisAngle(mat, out_axis, out_theta);
}

tMatrix cSimJoint::BuildWorldTrans() const
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

void cSimJoint::AddTau(const Eigen::VectorXd& tau)
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

const cSpAlg::tSpVec& cSimJoint::GetTau() const
{
	return mTotalTau;
}

void cSimJoint::ApplyTau()
{
	switch (mParams.mType)
	{
	case cKinTree::eJointTypeRevolute:
		ApplyTorque();
		break;
	case cKinTree::eJointTypePlanar:
		ApplyTorque();
		ApplyForce();
		break;
	case cKinTree::eJointTypePrismatic:
		ApplyForce();
		break;
	case cKinTree::eJointTypeFixed:
		break;
	case cKinTree::eJointTypeSpherical:
		ApplyTorque();
		break;
	default:
		assert(false); // unsupported joint type
		break;
	}
}

void cSimJoint::ClearTau()
{
	mTotalTau.setZero();
}

tVector cSimJoint::CalcWorldPos() const
{
	tVector world_pos;
	if (HasChild())
	{
		world_pos = mChild->LocalToWorldPos(GetChildPos());
	}
	else
	{
		assert(GetType() != cKinTree::eJointTypePrismatic
			&& GetType() != cKinTree::eJointTypePlanar);
		world_pos = mParent->LocalToWorldPos(GetParentPos());
	}
	return world_pos;
}

tVector cSimJoint::CalcWorldPos(const tVector& local_pos) const
{
	tVector world_pos = local_pos;
	world_pos[3] = 1;
	tMatrix trans = BuildWorldTrans();
	world_pos = trans * world_pos;
	world_pos[3] = 0;
	return world_pos;
}

tVector cSimJoint::CalcWorldVel() const
{
	if (HasChild())
	{
		tVector pos = GetChildPos();
		return mChild->GetLinearVelocity(pos);
	}
	else
	{
		tVector pos = GetParentPos();
		return mParent->GetLinearVelocity(pos);
	}
}

tVector cSimJoint::CalcWorldVel(const tVector& local_pos) const
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

tVector cSimJoint::CalcWorldAngVel() const
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

double cSimJoint::GetTorqueLimit() const
{
	return mParams.mTorqueLimit;
}

double cSimJoint::GetForceLimit() const
{
	return mParams.mForceLimit;
}

void cSimJoint::SetTorqueLimit(double lim)
{
	mParams.mTorqueLimit = lim;
}

void cSimJoint::SetForceLimit(double lim)
{
	mParams.mForceLimit = lim;
}

const tVector& cSimJoint::GetParentPos() const
{
	return mParams.mParentPos;
}

const tVector& cSimJoint::GetChildPos() const
{
	return mParams.mChildPos; 
}

const tQuaternion& cSimJoint::GetParentRot() const
{
	return mParams.mParentRot;
}

const tQuaternion& cSimJoint::GetChildRot() const
{
	return mParams.mChildRot;
}

const std::shared_ptr<cSimObj>& cSimJoint::GetParent() const
{
	return mParent;
}

const std::shared_ptr<cSimObj>& cSimJoint::GetChild() const
{
	return mChild;
}

void cSimJoint::ClampTotalTorque(tVector& out_torque) const
{
	double mag = out_torque.norm();
	double torque_lim = GetTorqueLimit();
	if (mag > torque_lim)
	{
		out_torque *= torque_lim / mag;
	}
}

void cSimJoint::ClampTotalForce(tVector& out_force) const
{
	double mag = out_force.norm();
	double force_lim = GetForceLimit();
	if (mag > force_lim)
	{
		out_force *= force_lim / mag;
	}
}

const tVector& cSimJoint::GetLimLow() const
{
	return mParams.mLimLow;
}

const tVector& cSimJoint::GetLimHigh() const
{
	return mParams.mLimHigh;
}

bool cSimJoint::HasJointLim() const
{
	return (mParams.mLimLow[0] <= mParams.mLimHigh[0]
		|| mParams.mLimLow[1] <= mParams.mLimHigh[1]
		|| mParams.mLimLow[2] <= mParams.mLimHigh[2]
		|| mParams.mLimLow[3] <= mParams.mLimHigh[3]);
}

bool cSimJoint::EnableAdjacentCollision() const
{
	return mParams.mEnableAdjacentCollision;
}

int cSimJoint::GetParamSize() const
{
	return cKinTree::GetJointParamSize(GetType());
}

void cSimJoint::BuildPose(Eigen::VectorXd& out_pose) const
{
	if (IsValid())
	{
		switch (mParams.mType)
		{
		case cKinTree::eJointTypeRevolute:
			BuildPoseRevolute(out_pose);
			break;
		case cKinTree::eJointTypePlanar:
			BuildPosePlanar(out_pose);
			break;
		case cKinTree::eJointTypePrismatic:
			BuildPosePristmatic(out_pose);
			break;
		case cKinTree::eJointTypeFixed:
			BuildPoseFixed(out_pose);
			break;
		case cKinTree::eJointTypeSpherical:
			BuildPoseSpherical(out_pose);
			break;
		default:
			assert(false); // unsupported joint type
			break;
		}
	}
	else
	{
		int param_size = GetParamSize();
		out_pose = Eigen::VectorXd::Zero(param_size);
	}
}

void cSimJoint::BuildVel(Eigen::VectorXd& out_vel) const
{
	if (IsValid())
	{
		switch (mParams.mType)
		{
		case cKinTree::eJointTypeRevolute:
			BuildVelRevolute(out_vel);
			break;
		case cKinTree::eJointTypePlanar:
			BuildVelPlanar(out_vel);
			break;
		case cKinTree::eJointTypePrismatic:
			BuildVelPristmatic(out_vel);
			break;
		case cKinTree::eJointTypeFixed:
			BuildVelFixed(out_vel);
			break;
		case cKinTree::eJointTypeSpherical:
			BuildVelSpherical(out_vel);
			break;
		default:
			assert(false); // unsupported joint type
			break;
		}
	}
	else
	{
		int param_size = GetParamSize();
		out_vel = Eigen::VectorXd::Zero(param_size);
	}
}

bool cSimJoint::HasMultBody() const
{
	assert(mCons == nullptr || mMultBodyCons == nullptr);
	return mMultBodyCons != nullptr;
}

void cSimJoint::Bind()
{
	if (!IsValid())
	{
		BuildConstraint(mWorld);
	}
}

void cSimJoint::Unbind()
{
	if (IsValid())
	{
		RemoveFromWorld();
	}
}

tVector cSimJoint::GetTotalTorque() const
{
	tVector torque = cSpAlg::GetOmega(mTotalTau);
	return torque;
}

void cSimJoint::RemoveFromWorld()
{
	if (mWorld != nullptr)
	{
		mWorld->RemoveJoint(*this);
		mCons.reset();
		mMultBodyCons.reset();
	}
}

tVector cSimJoint::CalcParentLocalPos(const tVector& local_pos) const
{
	cKinTree::eJointType joint_type = GetType();
	tVector pt = local_pos;
	if (joint_type == cKinTree::eJointTypePrismatic)
	{
		double delta = GetPrismaticOffset();
		pt[0] += delta;
	}

	tVector parent_pos = cMathUtil::QuatRotVec(GetParentRot(), pt);
	parent_pos += GetParentPos();
	return parent_pos;
}

tVector cSimJoint::CalcChildLocalPos(const tVector& local_pos) const
{
	tVector child_pos = cMathUtil::QuatRotVec(GetChildRot(), local_pos);
	child_pos += GetChildPos();
	return child_pos;
}

double cSimJoint::GetPrismaticOffset() const
{
	assert(mParams.mType == cKinTree::eJointTypePrismatic);
	double scale = mWorld->GetScale();
	double delta = 0;

	if (HasMultBody())
	{
		//const btMultiBodySliderConstraint* prismatic = reinterpret_cast<const btMultiBodySliderConstraint*>(mMultBodyCons.get());
		//delta = prismatic->getPosition(0);
		//delta /= scale;
		assert(false); // unsupported
	}
	else
	{
		const btSliderConstraint* prismatic = reinterpret_cast<const btSliderConstraint*>(mCons.get());
		delta = prismatic->getLinearPos();
		delta /= scale;
	}
	return delta;
}

void cSimJoint::BuildConstraint(std::shared_ptr<cWorld>& world)
{
	switch (GetType())
	{
	case cKinTree::eJointTypeRevolute:
		BuildConstraintRevolute(world);
		break;
	case cKinTree::eJointTypePlanar:
		BuildConstraintPlanar(world);
		break;
	case cKinTree::eJointTypePrismatic:
		BuildConstraintPrismatic(world);
		break;
	case cKinTree::eJointTypeFixed:
		BuildConstraintFixed(world);
		break;
	case cKinTree::eJointTypeSpherical:
		BuildConstraintSpherical(world);
		break;
	default:
		assert(false); // unsupported joint type
		break;
	}
	assert(mCons != nullptr ^ mMultBodyCons != nullptr);
	mWorld->AddJoint(*this);
}

void cSimJoint::BuildConstraintRevolute(std::shared_ptr<cWorld>& world)
{
	btScalar scale = static_cast<btScalar>(world->GetScale());
	auto sim_link = dynamic_cast<const cSimBodyLink*>(mParent.get());
	if (sim_link == nullptr)
	{
		tVector euler0 = cMathUtil::QuaternionToEuler(GetParentRot());
		tVector euler1 = cMathUtil::QuaternionToEuler(GetChildRot());

		btTransform anchor0_t;
		anchor0_t.setIdentity();

		const tVector& parent_pos = GetParentPos();
		btVector3 anchor0 = scale * btVector3(static_cast<btScalar>(parent_pos[0]),
									static_cast<btScalar>(parent_pos[1]),
									static_cast<btScalar>(parent_pos[2]));
		anchor0_t.setOrigin(anchor0);
		anchor0_t.getBasis().setEulerZYX(static_cast<btScalar>(euler0[0]), static_cast<btScalar>(euler0[1]), static_cast<btScalar>(euler0[2]));
		btTransform anchor1_t;
		anchor1_t.setIdentity();

		const tVector& child_pos = GetChildPos();
		btVector3 anchor1 = scale * btVector3(static_cast<btScalar>(child_pos[0]),
									static_cast<btScalar>(child_pos[1]),
									static_cast<btScalar>(child_pos[2]));
		anchor1_t.setOrigin(anchor1);
		anchor1_t.getBasis().setEulerZYX(static_cast<btScalar>(euler1[0]), static_cast<btScalar>(euler1[1]), static_cast<btScalar>(euler1[2]));

		btHingeConstraint* cons = nullptr;

		cSimRigidBody* parent = dynamic_cast<cSimRigidBody*>(mParent.get());
		cSimRigidBody* child = dynamic_cast<cSimRigidBody*>(mChild.get());
		cons = new btHingeConstraint(*(parent->GetSimBody()), *(child->GetSimBody()),
									anchor0_t, anchor1_t);

		// bullet limits are flipped
		cons->setLimit(-static_cast<btScalar>(mParams.mLimHigh[0]), -static_cast<btScalar>(mParams.mLimLow[0]));
		mCons = std::shared_ptr<btTypedConstraint>(cons);
	}
	else
	{
		assert(false); // unsupported
	}
}

void cSimJoint::BuildConstraintPlanar(std::shared_ptr<cWorld>& world)
{
	assert(false); // unsupported
}

void cSimJoint::BuildConstraintPrismatic(std::shared_ptr<cWorld>& world)
{
	btScalar scale = static_cast<btScalar>(world->GetScale());
	auto parent_sim_link = dynamic_cast<const cSimBodyLink*>(mParent.get());
	auto child_sim_link = dynamic_cast<const cSimBodyLink*>(mChild.get());

	if (parent_sim_link == nullptr)
	{
		const tVector& euler0 = cMathUtil::QuaternionToEuler(GetParentRot());
		const tVector& euler1 = cMathUtil::QuaternionToEuler(GetChildRot());

		btTransform anchor0_t;
		anchor0_t.setIdentity();
		
		const tVector& parent_pos = GetParentPos();
		btVector3 anchor0 = scale * btVector3(static_cast<btScalar>(parent_pos[0]),
									static_cast<btScalar>(parent_pos[1]),
									static_cast<btScalar>(parent_pos[2]));

		anchor0_t.setOrigin(anchor0);
		anchor0_t.getBasis().setEulerZYX(static_cast<btScalar>(euler0[0]), static_cast<btScalar>(euler0[1]), static_cast<btScalar>(euler0[2]));
		btTransform anchor1_t;
		anchor1_t.setIdentity();

		const tVector& child_pos = GetChildPos();
		btVector3 anchor1 = scale * btVector3(static_cast<btScalar>(child_pos[0]),
											static_cast<btScalar>(child_pos[1]),
											static_cast<btScalar>(child_pos[2]));
		anchor1_t.setOrigin(anchor1);
		anchor1_t.getBasis().setEulerZYX(static_cast<btScalar>(euler1[0]), static_cast<btScalar>(euler1[1]), static_cast<btScalar>(euler1[2]));

		btSliderConstraint* cons = nullptr;

		cSimRigidBody* parent = dynamic_cast<cSimRigidBody*>(mParent.get());
		cSimRigidBody* child = dynamic_cast<cSimRigidBody*>(mChild.get());
		cons = new btSliderConstraint(*(parent->GetSimBody()), *(child->GetSimBody()),
										anchor0_t, anchor1_t, true);

		cons->setLowerLinLimit(static_cast<btScalar>(scale * mParams.mLimLow[0]));
		cons->setUpperLinLimit(static_cast<btScalar>(scale * mParams.mLimHigh[0]));
		cons->setLowerAngLimit(0.0f);
		cons->setUpperAngLimit(0.0f);
		mCons = std::shared_ptr<btTypedConstraint>(cons);
	}
	else
	{
		assert(false); // unsupported
	}
}

void cSimJoint::BuildConstraintFixed(std::shared_ptr<cWorld>& world)
{
	btScalar scale = static_cast<btScalar>(world->GetScale());
	auto sim_link = dynamic_cast<const cSimBodyLink*>(mParent.get());
	if (sim_link == nullptr)
	{
		const tVector& euler0 = cMathUtil::QuaternionToEuler(GetParentRot());
		const tVector& euler1 = cMathUtil::QuaternionToEuler(GetChildRot());

		btTransform anchor0_t;
		anchor0_t.setIdentity();
		
		const tVector& parent_pos = GetParentPos();
		btVector3 anchor0 = scale * btVector3(static_cast<btScalar>(parent_pos[0]),
									static_cast<btScalar>(parent_pos[1]),
									static_cast<btScalar>(parent_pos[2]));
		anchor0_t.setOrigin(anchor0);
		anchor0_t.getBasis().setEulerZYX(static_cast<btScalar>(euler0[0]), static_cast<btScalar>(euler0[1]), static_cast<btScalar>(euler0[2]));
		btTransform anchor1_t;
		anchor1_t.setIdentity();

		const tVector& child_pos = GetChildPos();
		btVector3 anchor1 = scale * btVector3(static_cast<btScalar>(child_pos[0]),
									static_cast<btScalar>(child_pos[1]),
									static_cast<btScalar>(child_pos[2]));
		anchor1_t.setOrigin(anchor1);
		anchor1_t.getBasis().setEulerZYX(static_cast<btScalar>(euler1[0]), static_cast<btScalar>(euler1[1]), static_cast<btScalar>(euler1[2]));

		cSimRigidBody* parent = dynamic_cast<cSimRigidBody*>(mParent.get());
		cSimRigidBody* child = dynamic_cast<cSimRigidBody*>(mChild.get());

		btFixedConstraint* cons = nullptr;
		cons = new btFixedConstraint(*(parent->GetSimBody()), *(child->GetSimBody()), anchor0_t, anchor1_t);
		mCons = std::shared_ptr<btTypedConstraint>(cons);
	}
	else
	{
		assert(false); // unsupported
	}
}

void cSimJoint::BuildConstraintSpherical(std::shared_ptr<cWorld>& world)
{
	btScalar scale = static_cast<btScalar>(world->GetScale());
	auto parent_sim_link = dynamic_cast<const cSimBodyLink*>(mParent.get());
	auto child_sim_link = dynamic_cast<const cSimBodyLink*>(mChild.get());
	if (parent_sim_link == nullptr)
	{
		const tVector& euler0 = cMathUtil::QuaternionToEuler(GetParentRot());
		const tVector& euler1 = cMathUtil::QuaternionToEuler(GetChildRot());

		btTransform anchor0_t;
		anchor0_t.setIdentity();
		
		const tVector& parent_pos = GetParentPos();
		btVector3 anchor0 = scale * btVector3(static_cast<btScalar>(parent_pos[0]),
									static_cast<btScalar>(parent_pos[1]),
									static_cast<btScalar>(parent_pos[2]));
		anchor0_t.setOrigin(anchor0);
		anchor0_t.getBasis().setEulerZYX(static_cast<btScalar>(euler0[0]), static_cast<btScalar>(euler0[1]), static_cast<btScalar>(euler0[2]));
		btTransform anchor1_t;
		anchor1_t.setIdentity();

		const tVector& child_pos = GetChildPos();
		btVector3 anchor1 = scale * btVector3(static_cast<btScalar>(child_pos[0]),
									static_cast<btScalar>(child_pos[1]),
									static_cast<btScalar>(child_pos[2]));
		anchor1_t.setOrigin(anchor1);
		anchor1_t.getBasis().setEulerZYX(static_cast<btScalar>(euler1[0]), static_cast<btScalar>(euler1[1]), static_cast<btScalar>(euler1[2]));

		cSimRigidBody* parent = dynamic_cast<cSimRigidBody*>(mParent.get());
		cSimRigidBody* child = dynamic_cast<cSimRigidBody*>(mChild.get());
		
		btGeneric6DofSpring2Constraint* cons = nullptr;
		cons = new btGeneric6DofSpring2Constraint(*(parent->GetSimBody()), *(child->GetSimBody()), anchor0_t, anchor1_t);

		cons->setLinearUpperLimit(btVector3(0, 0, 0));
		cons->setLinearLowerLimit(btVector3(0, 0, 0));

		// bullet limits are flipped
		cons->setAngularUpperLimit(-btVector3(static_cast<btScalar>(mParams.mLimLow[0]),
												static_cast<btScalar>(mParams.mLimLow[1]),
												static_cast<btScalar>(mParams.mLimLow[2])));
		cons->setAngularLowerLimit(-btVector3(static_cast<btScalar>(mParams.mLimHigh[0]),
												static_cast<btScalar>(mParams.mLimHigh[1]),
												static_cast<btScalar>(mParams.mLimHigh[2])));
		mCons = std::shared_ptr<btTypedConstraint>(cons);
	}
	else if (child_sim_link == nullptr)
	{
		cSimBodyLink* parent_mult = dynamic_cast<cSimBodyLink*>(mParent.get());
		cSimRigidBody* child_rb = dynamic_cast<cSimRigidBody*>(mChild.get());

		assert(GetParentRot().isApprox(tQuaternion::Identity()));
		assert(GetChildRot().isApprox(tQuaternion::Identity()));

		const tVector& parent_pos = GetParentPos();
		const tVector& child_pos = GetChildPos();
		auto cons = new btMultiBodyPoint2Point(parent_mult->GetMultBody().get(), parent_mult->GetJointID(), child_rb->GetSimBody().get(), 
												btVector3(parent_pos[0], parent_pos[1], parent_pos[2]),
												btVector3(child_pos[0], child_pos[1], child_pos[2]));
		mMultBodyCons = std::shared_ptr<btMultiBodyConstraint>(cons);
	}
}


void cSimJoint::BuildPoseRevolute(Eigen::VectorXd& out_pose) const
{
	int param_size = GetParamSize();
	out_pose.resize(param_size);
	btHingeConstraint* hinge = reinterpret_cast<btHingeConstraint*>(mCons.get());
	double theta = -hinge->getHingeAngle();
	out_pose[0] = theta;
}

void cSimJoint::BuildPosePlanar(Eigen::VectorXd& out_pose) const
{
	int param_size = GetParamSize();
	out_pose.resize(param_size);
	assert(false); // unsupported
}

void cSimJoint::BuildPosePristmatic(Eigen::VectorXd& out_pose) const
{
	int param_size = GetParamSize();
	out_pose.resize(param_size);
	double delta = GetPrismaticOffset();
	out_pose[0] = delta;
}

void cSimJoint::BuildPoseFixed(Eigen::VectorXd& out_pose) const
{
	int param_size = GetParamSize();
	out_pose = Eigen::VectorXd::Zero(param_size);
}

void cSimJoint::BuildPoseSpherical(Eigen::VectorXd& out_pose) const
{
	int param_size = GetParamSize();
	tVector euler = tVector::Zero();
	if (HasMultBody())
	{
		assert(false); // unsupported
	}
	else
	{
		btGeneric6DofSpring2Constraint* sphere = reinterpret_cast<btGeneric6DofSpring2Constraint*>(mCons.get());
		euler = tVector(-sphere->getAngle(0), -sphere->getAngle(1), -sphere->getAngle(2), 0);
	}

	tQuaternion q = cMathUtil::EulerToQuaternion(euler);
	bool flip = (q.w() < 0);

	out_pose.resize(param_size);
	out_pose[0] = (flip) ? -q.w() : q.w();
	out_pose[1] = (flip) ? -q.x() : q.x();
	out_pose[2] = (flip) ? -q.y() : q.y();
	out_pose[3] = (flip) ? -q.z() : q.z();
}


void cSimJoint::BuildVelRevolute(Eigen::VectorXd& out_pose) const
{
	int param_size = GetParamSize();
	out_pose.resize(param_size);
	tVector vel = CalcLocalAngVel();
	out_pose[0] = vel[2];
}

void cSimJoint::BuildVelPlanar(Eigen::VectorXd& out_pose) const
{
	int param_size = GetParamSize();
	out_pose = std::numeric_limits<double>::quiet_NaN() * Eigen::VectorXd::Ones(param_size);
	assert(false); // unsupported
}

void cSimJoint::BuildVelPristmatic(Eigen::VectorXd& out_pose) const
{
	assert(mParams.mType == cKinTree::eJointTypePrismatic);
	int param_size = GetParamSize();
	out_pose.resize(param_size);
	tVector vel = CalcLocalLinVel();
	out_pose[0] = vel[0];
}

void cSimJoint::BuildVelFixed(Eigen::VectorXd& out_pose) const
{
	int param_size = GetParamSize();
	out_pose.resize(param_size);
}

void cSimJoint::BuildVelSpherical(Eigen::VectorXd& out_vel) const
{
	int param_size = GetParamSize();
	out_vel.resize(param_size);
	tVector vel = CalcLocalAngVel();
	out_vel.segment(0, param_size) = vel.segment(0, param_size);
}

void cSimJoint::SetTotalTorque(const tVector& torque)
{
	cSpAlg::SetOmega(torque, mTotalTau);
}

tVector cSimJoint::GetTotalForce() const
{
	tVector force = cSpAlg::GetV(mTotalTau);
	return force;
}

const std::shared_ptr<btTypedConstraint>& cSimJoint::GetCons() const
{
	return mCons;
}

const std::shared_ptr<btMultiBodyConstraint>& cSimJoint::GetMultBodyCons() const
{
	return mMultBodyCons;
}

void cSimJoint::SetTotalForce(const tVector& force)
{
	cSpAlg::SetV(force, mTotalTau);
}

void cSimJoint::ApplyTorque()
{
	tVector torque = GetTotalTorque();
	ClampTotalTorque(torque);
	SetTotalTorque(torque);

	tMatrix trans = BuildWorldTrans();
	torque = trans * torque;

	if (HasParent())
	{
		mParent->ApplyTorque(-torque);
	}
	mChild->ApplyTorque(torque);
}

void cSimJoint::ApplyForce()
{
	tVector force = GetTotalForce();
	ClampTotalForce(force);
	SetTotalForce(force);

	tMatrix trans = BuildWorldTrans();
	force = trans * force;

	if (HasParent())
	{
		tVector parent_pt = CalcParentLocalPos(tVector::Zero());
		mParent->ApplyForce(-force, parent_pt);
	}
	mChild->ApplyForce(force, GetChildPos());
}

tVector cSimJoint::CalcLocalAngVel() const
{
	tVector ang_velp = tVector::Zero();
	if (HasParent())
	{
		ang_velp = mParent->GetAngularVelocity();
	}

	tVector ang_velc = mChild->GetAngularVelocity();
	tVector vel = ang_velc - ang_velp;
	vel[3] = 0;

	tMatrix trans = BuildWorldTrans();
	vel = trans.transpose() * vel;
	vel[3] = 0;
	return vel;
}

tVector cSimJoint::CalcLocalLinVel() const
{
	tVector velp = tVector::Zero();
	if (HasParent())
	{
		tVector parent_pt = CalcParentLocalPos(tVector::Zero());
		velp = mParent->GetLinearVelocity(parent_pt);
	}

	tVector velc = mChild->GetLinearVelocity(GetChildPos());
	tVector delta_vel = velc - velp;
	delta_vel[3] = 0;

	tMatrix trans = BuildWorldTrans();
	delta_vel = trans.transpose() * delta_vel;
	delta_vel[3] = 0;
	return delta_vel;
}
