#include "SimCapsule.h"

cSimCapsule::tParams::tParams()
{
	mType = eTypeDynamic;
	mMass = 1;
	mFriction = 0.9;
	mPos.setZero();
	mVel.setZero();
	mHeight = 1;
	mRadius = 1;
	mAxis = tVector(0, 0, 1, 1);
	mTheta = 0;
}

cSimCapsule::cSimCapsule()
{
}

cSimCapsule::~cSimCapsule()
{
}

void cSimCapsule::Init(const std::shared_ptr<cWorld>& world, const tParams& params)
{
	mType = params.mType;
	btScalar mass = (params.mType == eTypeDynamic) ? static_cast<btScalar>(params.mMass) : 0;
	
	mColShape = std::unique_ptr<btCollisionShape>(world->BuildCapsuleShape(params.mRadius, params.mHeight));
	
	btVector3 inertia(0, 0, 0);
	mColShape->calculateLocalInertia(mass, inertia);
	btRigidBody::btRigidBodyConstructionInfo cons_info(mass, this, mColShape.get(), inertia);
	mSimBody = std::unique_ptr<btRigidBody>(new btRigidBody(cons_info));
	mSimBody->setFriction(static_cast<btScalar>(params.mFriction));

	cSimRigidBody::Init(world);
	SetPos(params.mPos);
	SetRotation(params.mAxis, params.mTheta);

}

double cSimCapsule::GetHeight() const
{
	tVector size = GetSize();
	return size[1] - size[0];
}

double cSimCapsule::GetRadius() const
{
	tVector size = GetSize();
	return 0.5 * size[0];
}

cShape::eShape cSimCapsule::GetShape() const
{
	return cShape::eShapeCapsule;
}

tVector cSimCapsule::GetSize() const
{
	return mWorld->GetSizeCapsule(*this);
}