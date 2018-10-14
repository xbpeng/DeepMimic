#include "SimPlane.h"

cSimPlane::tParams::tParams()
{
	mMass = 0;
	mFriction = 0.9;
	mNormal = tVector(0, 1, 0, 0);
	mOrigin = tVector::Zero();
}

cSimPlane::cSimPlane()
{
}

cSimPlane::~cSimPlane()
{
}

void cSimPlane::Init(const std::shared_ptr<cWorld>& world, const tParams& params)
{
	mType = eTypeStatic;
	mColShape = std::unique_ptr<btCollisionShape>(world->BuildPlaneShape(params.mNormal, params.mOrigin));

	btRigidBody::btRigidBodyConstructionInfo cons_info(0, this, mColShape.get(), btVector3(0, 0, 0));
	mSimBody = std::unique_ptr<btRigidBody>(new btRigidBody(cons_info));
	mSimBody->setFriction(static_cast<btScalar>(params.mFriction));

	cSimRigidBody::Init(world);
}


tVector cSimPlane::GetCoeffs() const
{
	return GetSize();
}

cShape::eShape cSimPlane::GetShape() const
{
	return cShape::eShapePlane;
}

tVector cSimPlane::GetSize() const
{
	return mWorld->GetSizePlane(*this);
}