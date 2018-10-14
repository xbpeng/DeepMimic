#include "SimRigidBody.h"

cSimRigidBody::cSimRigidBody()
{
}

cSimRigidBody::~cSimRigidBody()
{
	RemoveFromWorld();
}

void cSimRigidBody::SetPos(const tVector& pos)
{
	cSimObj::SetPos(pos);

	const auto& body = GetSimBody();
	if (body->isKinematicObject())
	{
		btMotionState* motion_state = body->getMotionState();
		btTransform trans = body->getWorldTransform();
		motion_state->setWorldTransform(trans);
	}
}

void cSimRigidBody::SetRotation(const tVector& axis, double theta)
{
	cSimObj::SetRotation(axis, theta);
}

void cSimRigidBody::SetRotation(const tQuaternion& q)
{
	cSimObj::SetRotation(q);

	const auto& body = GetSimBody();
	if (body->isKinematicObject())
	{
		btMotionState* motion_state = body->getMotionState();
		btTransform trans = body->getWorldTransform();
		motion_state->setWorldTransform(trans);
	}

	body->updateInertiaTensor();
}

tVector cSimRigidBody::GetLinearVelocity() const
{
	auto& body = GetSimBody();
	const btVector3& bt_vel = body->getLinearVelocity();
	tVector vel = tVector(bt_vel[0], bt_vel[1], bt_vel[2], 0);
	vel /= mWorld->GetScale();
	return vel;
}

tVector cSimRigidBody::GetLinearVelocity(const tVector& local_pos) const
{
	auto& body = GetSimBody();
	double scale = mWorld->GetScale();
	tMatrix3 rot_mat = GetLocalToWorldRotMat();
	tVector rel_pos = tVector::Zero();
	rel_pos.segment(0, 3) = rot_mat * local_pos.segment(0, 3);

	btVector3 bt_vel = body->getVelocityInLocalPoint(static_cast<btScalar>(scale) * btVector3(static_cast<btScalar>(rel_pos[0]),
												static_cast<btScalar>(rel_pos[1]),
												static_cast<btScalar>(rel_pos[2])));
	tVector vel = tVector(bt_vel[0], bt_vel[1], bt_vel[2], 0);
	vel /= scale;
	return vel;
}

void cSimRigidBody::SetLinearVelocity(const tVector& vel)
{
	auto& body = GetSimBody();
	btScalar scale = static_cast<btScalar>(mWorld->GetScale());
	body->setLinearVelocity(scale * btVector3(static_cast<btScalar>(vel[0]),
										static_cast<btScalar>(vel[1]),
										static_cast<btScalar>(vel[2])));
}

tVector cSimRigidBody::GetAngularVelocity() const
{
	auto& body = GetSimBody();
	const btVector3& vel = body->getAngularVelocity();
	return tVector(vel[0], vel[1], vel[2], 0);
}

void cSimRigidBody::SetAngularVelocity(const tVector& vel)
{
	auto& body = GetSimBody();
	body->setAngularVelocity(btVector3(static_cast<btScalar>(vel[0]), static_cast<btScalar>(vel[1]),
							static_cast<btScalar>(vel[2])));
}

void cSimRigidBody::SetDamping(double linear_damping, double angular_damping)
{
	const std::unique_ptr<btRigidBody>& body = GetSimBody();
	body->setDamping(static_cast<btScalar>(linear_damping), static_cast<btScalar>(angular_damping));
}

double cSimRigidBody::GetMass() const
{
	return 1 / mSimBody->getInvMass();
}

void cSimRigidBody::ApplyForce(const tVector& force)
{
	ApplyForce(force, tVector::Zero());
}

void cSimRigidBody::ApplyForce(const tVector& force, const tVector& local_pos)
{
	auto& body = GetSimBody();
	btVector3 bt_force = btVector3(static_cast<btScalar>(force[0]), 
									static_cast<btScalar>(force[1]), 
									static_cast<btScalar>(force[2]));

	btScalar scale = static_cast<btScalar>(mWorld->GetScale());
	tMatrix3 rot_mat = GetLocalToWorldRotMat();
	tVector rel_pos = tVector::Zero();
	rel_pos.segment(0, 3) = rot_mat * local_pos.segment(0, 3);

	btVector3 bt_pos = btVector3(static_cast<btScalar>(rel_pos[0]),
								static_cast<btScalar>(rel_pos[1]),
								static_cast<btScalar>(rel_pos[2]));

	bt_force *= scale;
	bt_pos *= scale;
	body->applyForce(bt_force, bt_pos);
}

void cSimRigidBody::ApplyTorque(const tVector& torque)
{
	auto& body = GetSimBody();
	btScalar scale = static_cast<btScalar>(mWorld->GetScale());
	body->applyTorque(scale * scale * btVector3(static_cast<btScalar>(torque[0]),
										static_cast<btScalar>(torque[1]),
										static_cast<btScalar>(torque[2])));
}

void cSimRigidBody::ClearForces()
{
	mSimBody->clearForces();
}

void cSimRigidBody::SetKinematicObject(bool is_kin)
{
	int col_flags = mSimBody->getCollisionFlags();
	if (is_kin)
	{
		col_flags |= btCollisionObject::CF_KINEMATIC_OBJECT;
	}
	else
	{
		col_flags &= ~btCollisionObject::CF_KINEMATIC_OBJECT;
	}
	mSimBody->setCollisionFlags(col_flags);
}

bool cSimRigidBody::IsKinematicObject() const
{
	return mSimBody->isKinematicObject();
}

void cSimRigidBody::DisableDeactivation()
{
	mSimBody->setActivationState(DISABLE_DEACTIVATION);
}

void cSimRigidBody::Constrain(const tVector& linear_factor, const tVector& angular_factor)
{
	mWorld->Constrain(*this, linear_factor, angular_factor);
}

bool cSimRigidBody::HasSimBody() const
{
	return mSimBody != nullptr;
}

const std::unique_ptr<btRigidBody>& cSimRigidBody::GetSimBody() const
{
	return mSimBody;
}

const btCollisionObject* cSimRigidBody::GetCollisionObject() const
{
	return mSimBody.get();
}

btCollisionObject* cSimRigidBody::GetCollisionObject()
{
	return mSimBody.get();
}

void cSimRigidBody::Init(const std::shared_ptr<cWorld>& world)
{
	RemoveFromWorld();
	mSimBody->setUserPointer(this);
	AddToWorld(world);
	DisableDeactivation();
}

void cSimRigidBody::AddToWorld(const std::shared_ptr<cWorld>& world)
{
	if (mWorld != nullptr)
	{
		RemoveFromWorld();
	}

	mWorld = world;
	mWorld->AddRigidBody(*this);
}

void cSimRigidBody::RemoveFromWorld()
{
	if (mWorld != nullptr && mSimBody != nullptr)
	{
		int num_cons = GetNumConstraints();
		for (int c = num_cons - 1; c >= 0; --c)
		{
			cWorld::tConstraintHandle cons = GetConstraint(c);
			mWorld->RemoveConstraint(cons);
		}

		mWorld->RemoveRigidBody(*this);
		mWorld.reset();
		mSimBody.reset();
		mColShape.reset();
	}
}

int cSimRigidBody::GetNumConstraints() const
{
	return mSimBody->getNumConstraintRefs();
}

cWorld::tConstraintHandle cSimRigidBody::GetConstraint(int c) const
{
	cWorld::tConstraintHandle handle;
	handle.mCons = mSimBody->getConstraintRef(c);
	return handle;
}
