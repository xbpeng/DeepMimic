#pragma once

#include <memory>
#include "sim/World.h"
#include "anim/Shape.h"

class cSimObj
{
public:
	enum eType
	{
		eTypeDynamic,
		eTypeStatic,
		eTypeMax
	};

	virtual ~cSimObj();

	virtual tVector GetPos() const;
	virtual void SetPos(const tVector& pos);
	virtual void GetRotation(tVector& out_axis, double& out_theta) const;
	virtual tQuaternion GetRotation() const;
	virtual void SetRotation(const tVector& axis, double theta);
	virtual void SetRotation(const tQuaternion& q);
	virtual tMatrix GetWorldTransform() const;
	virtual tMatrix GetLocalTransform() const;

	virtual tVector GetLinearVelocity() const = 0;
	virtual tVector GetLinearVelocity(const tVector& local_pos) const = 0;
	virtual void SetLinearVelocity(const tVector& vel) = 0;
	virtual tVector GetAngularVelocity() const = 0;
	virtual void SetAngularVelocity(const tVector& vel) = 0;

	virtual tVector WorldToLocalPos(const tVector& world_pos) const;
	virtual tVector LocalToWorldPos(const tVector& local_pos) const;
	virtual tMatrix3 GetLocalToWorldRotMat() const;
	virtual tVector CalcCOM() const;
	virtual tVector CalcCOMVel() const;

	virtual void RegisterContact();
	virtual void RegisterContact(int contact_flags, int filter_flags);
	virtual void UpdateContact(int contact_flags, int filter_flags);
	virtual const cContactManager::tContactHandle& GetContactHandle() const;
	virtual bool IsInContact() const;
	virtual const tEigenArr<cContactManager::tContactPt>& GetContactPts() const;

	virtual void ApplyForce(const tVector& force) = 0;
	virtual void ApplyForce(const tVector& force, const tVector& local_pos) = 0;
	virtual void ApplyTorque(const tVector& torque) = 0;
	virtual void ClearForces() = 0;

	virtual short GetColGroup() const;
	virtual void SetColGroup(short col_group);
	virtual short GetColMask() const;
	virtual void SetColMask(short col_mask);

	virtual eType GetType() const;
	virtual cShape::eShape GetShape() const;
	virtual tVector GetSize() const = 0;

	virtual void CalcAABB(tVector& out_min, tVector& out_max) const;

	virtual const btCollisionShape* GetCollisionShape() const;

protected:
	std::shared_ptr<cWorld> mWorld;
	std::unique_ptr<btCollisionShape> mColShape;
	cContactManager::tContactHandle mContactHandle;
	bool mEnableContactFall;

	eType mType;
	short mColGroup;
	short mColMask;

	cSimObj();

	virtual const std::shared_ptr<cWorld>& GetWorld() const;
	virtual const btCollisionObject* GetCollisionObject() const = 0;
	virtual btCollisionObject* GetCollisionObject() = 0;
};