#include "World.h"

#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"

#include "sim/SimBox.h"
#include "sim/SimCapsule.h"
#include "sim/SimPlane.h"
#include "sim/SimSphere.h"
#include "sim/SimCylinder.h"
#include "sim/SimBodyLink.h"
#include "sim/SimCharacter.h"
#include "sim/SimJoint.h"

cWorld::tParams::tParams()
{
	mNumSubsteps = 1;
	mScale = 1;
	mGravity = gGravity;
}

cWorld::tConstraintHandle::tConstraintHandle()
{
	mCons = nullptr;
}

bool cWorld::tConstraintHandle::IsValid() const
{
	return mCons != nullptr;
}

void cWorld::tConstraintHandle::Clear()
{
	delete mCons;
	mCons = nullptr;
}

cWorld::cWorld()
	: mContactManager(*this)
{
	mDefaultLinearDamping = 0;
	mDefaultAngularDamping = 0;
	mTimeStep = 0;
}

cWorld::~cWorld()
{
}

void cWorld::Init(const tParams& params)
{
	mParams = params;

	mBroadPhase = std::unique_ptr<btBroadphaseInterface>(new btDbvtBroadphase());
	mCollisionConfig = std::unique_ptr<btDefaultCollisionConfiguration>(new btDefaultCollisionConfiguration());
	mCollisionDispatcher = std::unique_ptr<btCollisionDispatcher>(new btCollisionDispatcher(mCollisionConfig.get()));
	btGImpactCollisionAlgorithm::registerAlgorithm(mCollisionDispatcher.get());

	auto solver = new btMultiBodyConstraintSolver();
	mSolver = std::unique_ptr<btSequentialImpulseConstraintSolver>(solver);
	mSimWorld = std::unique_ptr<btMultiBodyDynamicsWorld>(new btMultiBodyDynamicsWorld(mCollisionDispatcher.get(),
														mBroadPhase.get(), solver, mCollisionConfig.get()));

	btContactSolverInfo& info = mSimWorld->getSolverInfo();
	info.m_solverMode = SOLVER_SIMD | SOLVER_USE_2_FRICTION_DIRECTIONS | SOLVER_FRICTION_SEPARATE | SOLVER_USE_WARMSTARTING;

	SetGravity(params.mGravity);
	
	mContactManager.Init();
	mPerturbManager.Clear();
}

void cWorld::Reset()
{
	mTimeStep = 0;
	mContactManager.Reset();
	mPerturbManager.Clear();

	mSimWorld->clearForces();
	mSolver->reset();
	mBroadPhase->resetPool(mCollisionDispatcher.get());

	btOverlappingPairCache* pair_cache = mSimWorld->getBroadphase()->getOverlappingPairCache();
	btBroadphasePairArray& pair_array = pair_cache->getOverlappingPairArray();
	for (int i = 0; i < pair_array.size(); ++i)
	{
		pair_cache->cleanOverlappingPair(pair_array[i], mSimWorld->getDispatcher());
	}
}

void cWorld::Update(double time_elapsed)
{
	time_elapsed = std::max(0.0, time_elapsed);
	mPerturbManager.Update(time_elapsed);

	btScalar timestep = static_cast<btScalar>(time_elapsed);
	btScalar subtimestep = timestep / mParams.mNumSubsteps;
	mSimWorld->stepSimulation(timestep, mParams.mNumSubsteps, subtimestep);
	mTimeStep = subtimestep;

	mContactManager.Update();
}

void cWorld::AddRigidBody(cSimRigidBody& obj)
{
	const std::unique_ptr<btRigidBody>& body = obj.GetSimBody();

	short col_group = obj.GetColGroup();
	short col_mask = obj.GetColMask();
	col_mask |= cContactManager::gFlagRayTest;

	obj.SetDamping(mDefaultLinearDamping, mDefaultAngularDamping);
	mSimWorld->addRigidBody(body.get(), col_group, col_mask);

	Constrain(obj);
}

void cWorld::RemoveRigidBody(cSimRigidBody& obj)
{
	mSimWorld->removeRigidBody(obj.GetSimBody().get());
}

void cWorld::AddCollisionObject(btCollisionObject* col_obj, int col_filter_group, int col_filter_mask)
{
	col_filter_mask |= cContactManager::gFlagRayTest;
	mSimWorld->addCollisionObject(col_obj, col_filter_group, col_filter_mask);
}

void cWorld::RemoveCollisionObject(btCollisionObject* col_obj)
{
	mSimWorld->removeCollisionObject(col_obj);
}

void cWorld::AddCharacter(cSimCharacter& sim_char)
{
	sim_char.SetLinearDamping(mDefaultLinearDamping);
	sim_char.SetAngularDamping(mDefaultAngularDamping);
	mSimWorld->addMultiBody(sim_char.GetMultiBody().get());

	const auto& constraints = sim_char.GetConstraints();
	for (int c = 0; c < static_cast<int>(constraints.size()); ++c)
	{
		mSimWorld->addMultiBodyConstraint(constraints[c].get());
	}
}

void cWorld::RemoveCharacter(cSimCharacter& sim_char)
{
	const auto& constraints = sim_char.GetConstraints();
	for (int c = 0; c < static_cast<int>(constraints.size()); ++c)
	{
		mSimWorld->removeMultiBodyConstraint(constraints[c].get());
	}

	mSimWorld->removeMultiBody(sim_char.GetMultiBody().get());
}

void cWorld::Constrain(cSimRigidBody& obj)
{
	Constrain(obj, tVector::Ones(), tVector::Ones());
}

void cWorld::Constrain(cSimRigidBody& obj, const tVector& linear_factor, const tVector& angular_factor)
{
	auto& body = obj.GetSimBody();
	tVector lin_f = tVector::Ones();
	tVector ang_f = tVector::Ones();
	BuildConsFactor(lin_f, ang_f);

	lin_f = lin_f.cwiseProduct(linear_factor);
	ang_f = ang_f.cwiseProduct(angular_factor);

	body->setLinearFactor(btVector3(static_cast<btScalar>(lin_f[0]),
									static_cast<btScalar>(lin_f[1]),
									static_cast<btScalar>(lin_f[2])));
	body->setAngularFactor(btVector3(static_cast<btScalar>(ang_f[0]),
									static_cast<btScalar>(ang_f[1]),
									static_cast<btScalar>(ang_f[2])));
}

void cWorld::RemoveConstraint(tConstraintHandle& handle)
{
	if (handle.IsValid())
	{
		mSimWorld->removeConstraint(handle.mCons);
	}
	handle.Clear();
}

void cWorld::AddJoint(const cSimJoint& joint)
{
	const auto& cons = joint.GetCons();
	const auto& mult_body_cons = joint.GetMultBodyCons();
	assert(cons == nullptr || mult_body_cons == nullptr);
	if (cons != nullptr)
	{
		mSimWorld->addConstraint(cons.get(), joint.EnableAdjacentCollision());
	}

	if (mult_body_cons != nullptr)
	{
		mSimWorld->addMultiBodyConstraint(mult_body_cons.get());
	}
}

void cWorld::RemoveJoint(cSimJoint& joint)
{
	const auto& cons = joint.GetCons();
	const auto& mult_body_cons = joint.GetMultBodyCons();
	if (cons != nullptr)
	{
		mSimWorld->removeConstraint(cons.get());
	}

	if (mult_body_cons != nullptr)
	{
		mSimWorld->removeMultiBodyConstraint(mult_body_cons.get());
	}
}

void cWorld::BuildConsFactor(tVector& out_linear_factor, tVector& out_angular_factor)
{
	out_linear_factor = tVector::Ones();
	out_angular_factor = tVector::Ones();
}

void cWorld::SetGravity(const tVector& gravity)
{
	double scale = GetScale();
	mSimWorld->setGravity(btVector3(static_cast<btScalar>(gravity[0] * scale), 
									static_cast<btScalar>(gravity[1] * scale), 
									static_cast<btScalar>(gravity[2] * scale)));
}

cContactManager::tContactHandle cWorld::RegisterContact(int contact_flags, int filter_flags)
{
	return mContactManager.RegisterContact(contact_flags, filter_flags);
}

void cWorld::UpdateContact(const cContactManager::tContactHandle& handle)
{
	mContactManager.UpdateContact(handle);
}

const tEigenArr<cContactManager::tContactPt>& cWorld::GetContactPts(const cContactManager::tContactHandle& handle) const
{
	return mContactManager.GetContactPts(handle);
}

const cContactManager& cWorld::GetContactManager() const
{
	return mContactManager;
}

bool cWorld::IsInContact(const cContactManager::tContactHandle& handle) const
{
	return mContactManager.IsInContact(handle);
}

void cWorld::RayTest(const tVector& beg, const tVector& end, tRayTestResults& results) const
{
	btScalar scale = static_cast<btScalar>(GetScale());
	btVector3 bt_beg = scale * btVector3(static_cast<btScalar>(beg[0]),
								static_cast<btScalar>(beg[1]), 
								static_cast<btScalar>(beg[2]));
	btVector3 bt_end = scale * btVector3(static_cast<btScalar>(end[0]),
								static_cast<btScalar>(end[1]),
								static_cast<btScalar>(end[2]));
	btCollisionWorld::ClosestRayResultCallback ray_callback(bt_beg, bt_end);
	
	mSimWorld->rayTest(bt_beg, bt_end, ray_callback);
	
	results.clear();
	if (ray_callback.hasHit())
	{
		auto& obj = ray_callback.m_collisionObject;
		const auto& hit_pt = ray_callback.m_hitPointWorld;
		tRayTestResult result;
		result.mObj = static_cast<cSimObj*>(obj->getUserPointer());
		result.mHitPos = tVector(hit_pt[0], hit_pt[1], hit_pt[2], 0) / scale;

		results.push_back(result);
	}
}

void cWorld::AddPerturb(const tPerturb& perturb)
{
	mPerturbManager.AddPerturb(perturb);
}

const cPerturbManager& cWorld::GetPerturbManager() const
{
	return mPerturbManager;
}

tVector cWorld::GetGravity() const
{
	double scale = GetScale();
	btVector3 bt_gravity = mSimWorld->getGravity();
	return tVector(bt_gravity[0], bt_gravity[1], bt_gravity[2], 0) / scale;
}

double cWorld::GetScale() const
{
	return mParams.mScale;
}

double cWorld::GetTimeStep() const
{
	return mTimeStep;
}

void cWorld::SetDefaultLinearDamping(double damping)
{
	mDefaultLinearDamping = damping;
}

double cWorld::GetDefaultLinearDamping() const
{
	return mDefaultLinearDamping;
}

void cWorld::SetDefaultAngularDamping(double damping)
{
	mDefaultAngularDamping = damping;
}

double cWorld::GetDefaultAngularDamping() const
{
	return mDefaultAngularDamping;
}

btBoxShape* cWorld::BuildBoxShape(const tVector& box_size) const
{
	btScalar scale = static_cast<btScalar>(GetScale());
	return new btBoxShape(scale * btVector3(static_cast<btScalar>(box_size[0] * 0.5),
							static_cast<btScalar>(box_size[1] * 0.5),
							static_cast<btScalar>(box_size[2] * 0.5)));
}

btCapsuleShape* cWorld::BuildCapsuleShape(double radius, double height) const
{
	btScalar scale = static_cast<btScalar>(GetScale());
	return new btCapsuleShape(static_cast<btScalar>(scale * radius),
								static_cast<btScalar>(scale * height));
}

btStaticPlaneShape* cWorld::BuildPlaneShape(const tVector& normal, const tVector& origin) const
{
	btVector3 bt_normal = btVector3(static_cast<btScalar>(normal[0]),
									static_cast<btScalar>(normal[1]),
									static_cast<btScalar>(normal[2]));
	btVector3 bt_origin = btVector3(static_cast<btScalar>(origin[0]),
									static_cast<btScalar>(origin[1]),
									static_cast<btScalar>(origin[2]));
	bt_normal.normalize();
	double scale = GetScale();
	btScalar w = static_cast<btScalar>(scale * bt_normal.dot(bt_origin));
	
	return new btStaticPlaneShape(bt_normal, w);
}

btSphereShape* cWorld::BuildSphereShape(double radius) const
{
	btScalar scale = static_cast<btScalar>(GetScale());
	return new btSphereShape(static_cast<btScalar>(scale * radius));
}

btCylinderShape* cWorld::BuildCylinderShape(double radius, double height) const
{
	btScalar scale = static_cast<btScalar>(GetScale());
	return new btCylinderShape(btVector3(static_cast<btScalar>(scale * radius),
										static_cast<btScalar>(0.5 * scale * height),
										static_cast<btScalar>(scale * radius)));
}

tVector cWorld::GetSizeBox(const cSimObj& obj) const
{
	assert(obj.GetShape() == cShape::eShapeBox);
	const btBoxShape* box_shape = reinterpret_cast<const btBoxShape*>(obj.GetCollisionShape());
	btVector3 half_len = box_shape->getHalfExtentsWithMargin();
	double scale = GetScale();
	return tVector(half_len[0] * 2, half_len[1] * 2, half_len[2] * 2, 0) / scale;
}

tVector cWorld::GetSizeCapsule(const cSimObj& obj) const
{
	assert(obj.GetShape() == cShape::eShapeCapsule);
	const btCapsuleShape* shape = reinterpret_cast<const btCapsuleShape*>(obj.GetCollisionShape());
	double scale = GetScale();
	double r = shape->getRadius();
	double h = shape->getHalfHeight();
	r /= scale;
	h /= scale;
	return tVector(2 * r, 2 * (h + r), 2 * r, 0);
}

tVector cWorld::GetSizePlane(const cSimObj& obj) const
{
	assert(obj.GetShape() == cShape::eShapePlane);
	const btStaticPlaneShape* shape = reinterpret_cast<const btStaticPlaneShape*>(obj.GetCollisionShape());
	double scale = GetScale();
	btVector3 n = shape->getPlaneNormal();
	btScalar c = shape->getPlaneConstant();
	return tVector(n[0], n[1], n[2], c / scale);
}

tVector cWorld::GetSizeSphere(const cSimObj& obj) const
{
	assert(obj.GetShape() == cShape::eShapeSphere);
	const btSphereShape* ball_shape = reinterpret_cast<const btSphereShape*>(obj.GetCollisionShape());
	double r = ball_shape->getRadius();
	double scale = GetScale();
	r /= scale;
	double d = 2 * r;
	return tVector(d, d, d, 0);
}

tVector cWorld::GetSizeCylinder(const cSimObj& obj) const
{
	assert(obj.GetShape() == cShape::eShapeCylinder);
	const btCylinderShape* shape = reinterpret_cast<const btCylinderShape*>(obj.GetCollisionShape());
	double scale = GetScale();
	double r = shape->getRadius();
	double h = 2 * shape->getHalfExtentsWithMargin()[1];
	r /= scale;
	h /= scale;
	return tVector(2 * r, h, 2 * r, 0);
}

std::unique_ptr<btMultiBodyDynamicsWorld>& cWorld::GetInternalWorld()
{
	return mSimWorld;
}

const std::unique_ptr<btMultiBodyDynamicsWorld>& cWorld::GetInternalWorld() const
{
	return mSimWorld;
}

int cWorld::GetNumConstriants() const
{
	return static_cast<int>(mSimWorld->getNumConstraints());
}