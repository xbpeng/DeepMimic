#include "SimSphere.h"

cSimSphere::tParams::tParams()
{
	mType = eTypeDynamic;
	mMass = 1;
	mFriction = 0.9;
	mPos.setZero();
	mRadius = 1;
	mAxis = tVector(0, 0, 1, 1);
	mVel.setZero();
	mTheta = 0;
}

cSimSphere::cSimSphere()
{
}

cSimSphere::~cSimSphere()
{
}

void cSimSphere::Init(const std::shared_ptr<cWorld>& world, const tParams& params)
{
	mType = params.mType;
	btScalar mass = (params.mType == eTypeDynamic) ? static_cast<btScalar>(params.mMass) : 0;

	mColShape = std::unique_ptr<btCollisionShape>(world->BuildSphereShape(params.mRadius));

	btVector3 inertia(0, 0, 0);
	mColShape->calculateLocalInertia(mass, inertia);
	btRigidBody::btRigidBodyConstructionInfo cons_info(mass, this, mColShape.get(), inertia);
	mSimBody = std::unique_ptr<btRigidBody>(new btRigidBody(cons_info));
	mSimBody->setFriction(static_cast<btScalar>(params.mFriction));
	
	cSimRigidBody::Init(world);
	SetPos(params.mPos);
	SetLinearVelocity(params.mVel);
	SetRotation(params.mAxis, params.mTheta);
}

double cSimSphere::GetRadius() const
{
	tVector size = GetSize();
	return 0.5 * size[0];
}

tVector cSimSphere::GetSize() const
{
	return mWorld->GetSizeSphere(*this);
}

cShape::eShape cSimSphere::GetShape() const
{
	return cShape::eShapeSphere;
}