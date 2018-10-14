#include "SimCylinder.h"

cSimCylinder::tParams::tParams()
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

cSimCylinder::cSimCylinder()
{
}

cSimCylinder::~cSimCylinder()
{
}

void cSimCylinder::Init(const std::shared_ptr<cWorld>& world, const tParams& params)
{
	mType = params.mType;
	btScalar mass = (params.mType == eTypeDynamic) ? static_cast<btScalar>(params.mMass) : 0;

	mColShape = std::unique_ptr<btCollisionShape>(world->BuildCylinderShape(params.mRadius, params.mHeight));

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

double cSimCylinder::GetHeight() const
{
	tVector size = GetSize();
	return size[1];
}

double cSimCylinder::GetRadius() const
{
	tVector size = GetSize();
	return 0.5 * size[0];
}

tVector cSimCylinder::GetSize() const
{
	return mWorld->GetSizeCylinder(*this);
}

cShape::eShape cSimCylinder::GetShape() const
{
	return cShape::eShapeCylinder;
}