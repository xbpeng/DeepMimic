#pragma once

#include "sim/SimObj.h"

class cSimRigidBody : public cSimObj, public btDefaultMotionState
{
public:
	virtual ~cSimRigidBody();

	virtual void SetPos(const tVector& pos);
	virtual void SetRotation(const tVector& axis, double theta);
	virtual void SetRotation(const tQuaternion& q);
	virtual tVector GetLinearVelocity() const;
	virtual tVector GetLinearVelocity(const tVector& local_pos) const;
	virtual void SetLinearVelocity(const tVector& vel);
	virtual tVector GetAngularVelocity() const;
	virtual void SetAngularVelocity(const tVector& vel);

	virtual void SetDamping(double linear_damping, double angular_damping);
	virtual double GetMass() const;

	virtual void ApplyForce(const tVector& force);
	virtual void ApplyForce(const tVector& force, const tVector& local_pos);
	virtual void ApplyTorque(const tVector& torque);
	virtual void ClearForces();

	virtual void SetKinematicObject(bool is_kin);
	virtual bool IsKinematicObject() const;

	virtual void DisableDeactivation();
	virtual void Constrain(const tVector& linear_factor, const tVector& angular_factor);

	virtual bool HasSimBody() const;
	virtual const std::unique_ptr<btRigidBody>& GetSimBody() const;

protected:
	std::unique_ptr<btRigidBody> mSimBody;
	
	cSimRigidBody();

	virtual void Init(const std::shared_ptr<cWorld>& world);
	virtual void AddToWorld(const std::shared_ptr<cWorld>& world);
	virtual void RemoveFromWorld();

	virtual const btCollisionObject* GetCollisionObject() const;
	virtual btCollisionObject* GetCollisionObject();

	virtual int GetNumConstraints() const;
	virtual cWorld::tConstraintHandle GetConstraint(int c) const;
};