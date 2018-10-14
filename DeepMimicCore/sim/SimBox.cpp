#include "SimBox.h"

cSimBox::tParams::tParams()
{
	mType = eTypeDynamic;
	mMass = 1;
	mFriction = 0.9;
	mPos = tVector(0, 0, 0, 0);
	mSize = tVector(1, 1, 1, 0);
	mAxis = tVector(0, 0, 1, 1);
	mVel.setZero();
	mTheta = 0;
}

cSimBox::cSimBox()
{
}

cSimBox::~cSimBox()
{
}

void cSimBox::Init(const std::shared_ptr<cWorld>& world, const tParams& params)
{
	mType = params.mType;
	btScalar mass = (params.mType == eTypeDynamic) ? static_cast<btScalar>(params.mMass) : 0;

	mColShape = std::unique_ptr<btCollisionShape>(world->BuildBoxShape(params.mSize));

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

tVector cSimBox::GetSize() const
{
	return mWorld->GetSizeBox(*this);
}

cShape::eShape cSimBox::GetShape() const
{
	return cShape::eShapeBox;
}