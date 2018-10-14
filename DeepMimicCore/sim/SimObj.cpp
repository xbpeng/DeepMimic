#include "SimObj.h"

cSimObj::cSimObj()
	: mWorld(nullptr)
{
	mType = eTypeDynamic;
	mColGroup = cContactManager::gFlagAll;
	mColMask = cContactManager::gFlagAll;
}

cSimObj::~cSimObj()
{
}

tVector cSimObj::GetPos() const
{
	const btCollisionObject* col_obj = GetCollisionObject();
	btTransform trans = col_obj->getWorldTransform();
	btVector3 origin = trans.getOrigin();
	tVector pos = tVector(origin[0], origin[1], origin[2], 0);
	pos /= mWorld->GetScale();
	return pos;
}

void cSimObj::SetPos(const tVector& pos)
{
	btCollisionObject* col_obj = GetCollisionObject();
	btTransform trans = col_obj->getWorldTransform();

	btScalar scale = static_cast<btScalar>(mWorld->GetScale());
	trans.setOrigin(scale * btVector3(static_cast<btScalar>(pos[0]),
		static_cast<btScalar>(pos[1]),
		static_cast<btScalar>(pos[2])));

	col_obj->setWorldTransform(trans);
}

void cSimObj::GetRotation(tVector& out_axis, double& out_theta) const
{
	const btCollisionObject* col_obj = GetCollisionObject();
	btTransform trans = col_obj->getWorldTransform();
	btQuaternion bt_q = trans.getRotation();
	btVector3 axis = bt_q.getAxis();

	out_theta = bt_q.getAngle();
	out_axis[0] = axis[0];
	out_axis[1] = axis[1];
	out_axis[2] = axis[2];
	out_axis[3] = 0;
}

tQuaternion cSimObj::GetRotation() const
{
	const btCollisionObject* col_obj = GetCollisionObject();
	btTransform trans = col_obj->getWorldTransform();
	btQuaternion bt_q = trans.getRotation();
	tQuaternion q = tQuaternion(bt_q.w(), bt_q.x(), bt_q.y(), bt_q.z());
	return q;
}

void cSimObj::SetRotation(const tVector& axis, double theta)
{
	tQuaternion q = cMathUtil::AxisAngleToQuaternion(axis, theta);
	SetRotation(q);
}

void cSimObj::SetRotation(const tQuaternion& q)
{
	btCollisionObject* col_obj = GetCollisionObject();
	btTransform trans = col_obj->getWorldTransform();
	btQuaternion bt_q = btQuaternion(static_cast<btScalar>(q.x()),
		static_cast<btScalar>(q.y()),
		static_cast<btScalar>(q.z()),
		static_cast<btScalar>(q.w()));
	trans.setRotation(bt_q);
	col_obj->setWorldTransform(trans);
}

tMatrix cSimObj::GetWorldTransform() const
{
	const btCollisionObject* col_obj = GetCollisionObject();
	const btTransform& bt_trans = col_obj->getWorldTransform();
	const btMatrix3x3& basis = bt_trans.getBasis();
	const btVector3& origin = bt_trans.getOrigin();
	double scale = mWorld->GetScale();

	tMatrix trans = tMatrix::Identity();
	for (int i = 0; i < 3; ++i)
	{
		auto curr_row = trans.row(i);
		auto bt_row = basis.getRow(i);
		for (int j = 0; j < 3; ++j)
		{
			curr_row[j] = bt_row[j];
		}
		curr_row[3] = origin[i] / scale;
	}
	return trans;
}

tMatrix cSimObj::GetLocalTransform() const
{
	tMatrix trans = GetWorldTransform();
	trans = cMathUtil::InvRigidMat(trans);
	return trans;
}

tVector cSimObj::WorldToLocalPos(const tVector& world_pos) const
{
	tMatrix world_to_local = GetLocalTransform();
	tVector local_pt = world_pos;

	local_pt[3] = 1;
	local_pt = world_to_local * local_pt;
	local_pt[3] = 0;

	return local_pt;
}

tVector cSimObj::LocalToWorldPos(const tVector& local_pos) const
{
	tMatrix local_to_world = GetWorldTransform();
	tVector world_pos = local_pos;

	world_pos[3] = 1;
	world_pos = local_to_world * world_pos;
	world_pos[3] = 0;

	return world_pos;
}

tMatrix3 cSimObj::GetLocalToWorldRotMat() const
{
	tMatrix local_to_world = GetWorldTransform();
	tMatrix3 mat = local_to_world.block(0, 0, 3, 3);
	return mat;
}

tVector cSimObj::CalcCOM() const
{
	return GetPos();
}

tVector cSimObj::CalcCOMVel() const
{
	return GetLinearVelocity();
}

void cSimObj::RegisterContact()
{
	RegisterContact(cContactManager::gFlagAll, cContactManager::gFlagAll);
}

void cSimObj::RegisterContact(int contact_flags, int filter_flags)
{
	if (!mContactHandle.IsValid())
	{
		mContactHandle = mWorld->RegisterContact(contact_flags, filter_flags);
		assert(mContactHandle.IsValid());
	}
	else
	{
		assert(false); // already registered contact
	}
}

void cSimObj::UpdateContact(int contact_flags, int filter_flags)
{
	mContactHandle.mFlags = contact_flags;
	mContactHandle.mFilterFlags = filter_flags;

	if (mContactHandle.IsValid())
	{
		mWorld->UpdateContact(mContactHandle);
	}
}

const cContactManager::tContactHandle& cSimObj::GetContactHandle() const
{
	return mContactHandle;
}

bool cSimObj::IsInContact() const
{
	bool in_contact = mWorld->IsInContact(mContactHandle);
	return in_contact;
}

const tEigenArr<cContactManager::tContactPt>& cSimObj::GetContactPts() const
{
	return mWorld->GetContactPts(mContactHandle);
}

short cSimObj::GetColGroup() const
{
	return mColGroup;
}

void cSimObj::SetColGroup(short col_group)
{
	mColGroup = col_group;
}

short cSimObj::GetColMask() const
{
	return mColMask;
}

void cSimObj::SetColMask(short col_mask)
{
	mColMask = col_mask;
}

cSimObj::eType cSimObj::GetType() const
{
	return mType;
}

cShape::eShape cSimObj::GetShape() const
{
	return cShape::eShapeNull;
}

void cSimObj::CalcAABB(tVector& out_min, tVector& out_max) const
{
	const auto* obj = GetCollisionObject();
	const auto* shape = GetCollisionShape();

	btVector3 bt_min;
	btVector3 bt_max;
	shape->getAabb(obj->getWorldTransform(), bt_min, bt_max);
	double scale = mWorld->GetScale();

	out_min = tVector(bt_min[0], bt_min[1], bt_min[2], 0) / scale;
	out_max = tVector(bt_max[0], bt_max[1], bt_max[2], 0) / scale;
}

const btCollisionShape* cSimObj::GetCollisionShape() const
{
	return mColShape.get();
}

const std::shared_ptr<cWorld>& cSimObj::GetWorld() const
{
	return mWorld;
}