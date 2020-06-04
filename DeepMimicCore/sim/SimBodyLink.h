#pragma once

#include "sim/SimObj.h"

class cSimBodyLink : public cSimObj, public btDefaultMotionState
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	struct tParams
	{
		double mMass;
		int mJointID;

		tParams();
	};

	cSimBodyLink();
	virtual ~cSimBodyLink();

	virtual void Init(const std::shared_ptr<cWorld>& world, const std::shared_ptr<cMultiBody>& mult_body, const tParams& params);
	virtual tVector GetSize() const override;
	
	virtual tVector GetLinearVelocity() const;
	virtual tVector GetLinearVelocity(const tVector& local_pos) const;
	virtual void SetLinearVelocity(const tVector& vel);
	virtual tVector GetAngularVelocity() const;
	virtual void SetAngularVelocity(const tVector& vel);

	virtual double GetMass() const;
	virtual double GetFriction() const;
	virtual void SetFriction(double friction);

	virtual void ApplyForce(const tVector& force);
	virtual void ApplyForce(const tVector& force, const tVector& local_pos);
	virtual void ApplyTorque(const tVector& torque);
	virtual void ClearForces();

	virtual cShape::eShape GetShape() const;
	virtual void UpdateVel(const tVector& lin_vel, const tVector& ang_vel);
	virtual const std::shared_ptr<cMultiBody>& GetMultBody() const;
	virtual int GetJointID() const;

protected:
	std::shared_ptr<cMultiBody> mMultBody;
	std::unique_ptr<btMultiBodyLinkCollider> mColObj;

	int mJointID;
	double mMass;
	tVector mSize;
	cShape::eShape mObjShape;

	tVector mLinVel;
	tVector mAngVel;

	virtual void InitSize(tVector& out_size) const;
	virtual cShape::eShape FetchObjShape() const;

	virtual void RemoveFromWorld();

	virtual const btCollisionObject* GetCollisionObject() const;
	virtual btCollisionObject* GetCollisionObject();
};
