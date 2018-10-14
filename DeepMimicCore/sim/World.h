#pragma once

#include <memory>

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"

#include "sim/MultiBody.h"
#include "sim/ContactManager.h"
#include "sim/PerturbManager.h"
#include "anim/KinTree.h"
#include "util/MathUtil.h"

class cSimObj;
class cSimJoint;
class cSimBodyLink;
class cSimRigidBody;
class cSimBox;
class cSimCapsule;
class cSimPlane;
class cSimSphere;
class cSimCharacter;

class cWorld : public std::enable_shared_from_this<cWorld>
{
public:
	enum eContactFlag
	{
		eContactFlagNone = 0,
		eContactFlagCharacter = 0x1,
		eContactFlagEnvironment = 0x1 << 1,
		eContactFlagObject = 0x1 << 2,
		eContactFlagAll = cContactManager::gFlagAll
	};

	struct tParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		tParams();
		int mNumSubsteps;
		double mScale;
		tVector mGravity;
	};

	struct tConstraintHandle
	{
		tConstraintHandle();
		bool IsValid() const;
		void Clear();
		btTypedConstraint* mCons;
	};

	struct tRayTestResult
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		cSimObj* mObj;
		tVector mHitPos;
	};
	typedef tEigenArr<tRayTestResult> tRayTestResults;

	cWorld();
	virtual ~cWorld();
	virtual void Init(const tParams& params);
	virtual void Reset();
	virtual void Update(double time_elapsed);

	virtual void AddRigidBody(cSimRigidBody& obj);
	virtual void RemoveRigidBody(cSimRigidBody& obj);

	virtual void AddCollisionObject(btCollisionObject* col_obj, int col_filter_group, int col_filter_mask);
	virtual void RemoveCollisionObject(btCollisionObject* col_obj);
	virtual void AddCharacter(cSimCharacter& sim_char);
	virtual void RemoveCharacter(cSimCharacter& sim_char);

	virtual void Constrain(cSimRigidBody& obj);
	virtual void Constrain(cSimRigidBody& obj, const tVector& linear_factor, const tVector& angular_factor);
	virtual void RemoveConstraint(tConstraintHandle& handle);
	virtual void AddJoint(const cSimJoint& joint);
	virtual void RemoveJoint(cSimJoint& joint);

	virtual void SetGravity(const tVector& gravity);

	virtual cContactManager::tContactHandle RegisterContact(int contact_flags, int filter_flags);
	virtual void UpdateContact(const cContactManager::tContactHandle& handle);
	virtual bool IsInContact(const cContactManager::tContactHandle& handle) const;
	virtual const tEigenArr<cContactManager::tContactPt>& GetContactPts(const cContactManager::tContactHandle& handle) const;
	virtual const cContactManager& GetContactManager() const;

	virtual void RayTest(const tVector& beg, const tVector& end, tRayTestResults& results) const;
	virtual void AddPerturb(const tPerturb& perturb);
	virtual const cPerturbManager& GetPerturbManager() const;

	virtual tVector GetGravity() const;
	virtual double GetScale() const;
	virtual double GetTimeStep() const;
	virtual void SetDefaultLinearDamping(double damping);
	virtual double GetDefaultLinearDamping() const;
	virtual void SetDefaultAngularDamping(double damping);
	virtual double GetDefaultAngularDamping() const;

	// yuck, avoid using this
	std::unique_ptr<btMultiBodyDynamicsWorld>& GetInternalWorld();
	const std::unique_ptr<btMultiBodyDynamicsWorld>& GetInternalWorld() const;

	// object interface
	virtual btBoxShape* BuildBoxShape(const tVector& box_sizee) const;
	virtual btCapsuleShape* BuildCapsuleShape(double radius, double height) const;
	virtual btStaticPlaneShape* BuildPlaneShape(const tVector& normal, const tVector& origin) const;
	virtual btSphereShape* BuildSphereShape(double radius) const;
	virtual btCylinderShape* BuildCylinderShape(double radius, double height) const;

	virtual tVector GetSizeBox(const cSimObj& obj) const;
	virtual tVector GetSizeCapsule(const cSimObj& obj) const;
	virtual tVector GetSizePlane(const cSimObj& obj) const;
	virtual tVector GetSizeSphere(const cSimObj& obj) const;
	virtual tVector GetSizeCylinder(const cSimObj& obj) const;
	
protected:
	struct tConstraintEntry
	{
		cSimObj* mObj0;
		cSimObj* mObj1;
	};

	tParams mParams;
	double mTimeStep;
	double mDefaultLinearDamping;
	double mDefaultAngularDamping;
	std::unique_ptr<btMultiBodyDynamicsWorld> mSimWorld;

	std::unique_ptr<btConstraintSolver> mSolver;
	std::unique_ptr<btCollisionDispatcher> mCollisionDispatcher;
	std::unique_ptr<btDefaultCollisionConfiguration> mCollisionConfig;
	std::unique_ptr<btBroadphaseInterface> mBroadPhase;

	cContactManager mContactManager;
	cPerturbManager mPerturbManager;
	
	virtual int GetNumConstriants() const;
	virtual void BuildConsFactor(tVector& out_linear_factor, tVector& out_angular_factor);
};
