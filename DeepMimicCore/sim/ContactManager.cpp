#include "ContactManager.h"
#include "sim/World.h"
#include "SimObj.h"
#include <iostream>

const int cContactManager::gInvalidID = -1;

cContactManager::tContactPt::tContactPt()
{
	mPos.setZero();
	mForce.setZero();
}

cContactManager::tContactHandle::tContactHandle()
{
	mID = gInvalidID;
	mFlags = gFlagAll;
	mFilterFlags = gFlagAll;
}

cContactManager::tContactEntry::tContactEntry()
{
	mFlags = gFlagAll;
	mFilterFlags = gFlagAll;
}

bool cContactManager::tContactHandle::IsValid() const
{
	return mID != gInvalidID;
}

cContactManager::cContactManager(cWorld& world)
	: mWorld(world)
{

}

cContactManager::~cContactManager()
{

}

void cContactManager::Init()
{
	Clear();
}

void cContactManager::Reset()
{
	for (int i = 0; i < GetNumEntries(); ++i)
	{
		tContactEntry& entry = mContactEntries[i];
		entry.mContactPts.clear();
	}
}

void cContactManager::Clear()
{
	mContactEntries.clear();
}

void cContactManager::Update()
{
	ClearContacts();
	double world_scale = mWorld.GetScale();
	double timestep = mWorld.GetTimeStep();
	std::unique_ptr<btMultiBodyDynamicsWorld>& bt_world = mWorld.GetInternalWorld();

	int num_manifolds = bt_world->getDispatcher()->getNumManifolds();
	for (int i = 0; i < num_manifolds; ++i)
	{
		btPersistentManifold* mani = bt_world->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject* obj0 = static_cast<const btCollisionObject*>(mani->getBody0());
		const btCollisionObject* obj1 = static_cast<const btCollisionObject*>(mani->getBody1());

		int num_contacts = mani->getNumContacts();
		for (int j = 0; j < num_contacts; ++j)
		{
			btManifoldPoint& pt = mani->getContactPoint(j);
			btScalar dist_tol = static_cast<btScalar>(0.001 * world_scale);

			if (pt.getDistance() <= dist_tol)
			{
				const cSimObj* sim_obj0 = static_cast<const cSimObj*>(obj0->getUserPointer());
				const cSimObj* sim_obj1 = static_cast<const cSimObj*>(obj1->getUserPointer());

				const tContactHandle& h0 = sim_obj0->GetContactHandle();
				const tContactHandle& h1 = sim_obj1->GetContactHandle();
			
				bool valid_contact = IsValidContact(h0, h1);
				if (valid_contact)
				{
					double impulse = pt.getAppliedImpulse() / world_scale;
					tVector normal = tVector(pt.m_normalWorldOnB[0], pt.m_normalWorldOnB[1], pt.m_normalWorldOnB[2], 0);
					tVector force0 = (impulse / timestep) * normal;

					if (h0.IsValid())
					{
						tContactPt pt0; 
						const btVector3& contact_pt = pt.getPositionWorldOnA();
						pt0.mPos = tVector(contact_pt[0], contact_pt[1], contact_pt[2], 0) / world_scale;
						pt0.mForce = force0;
						mContactEntries[h0.mID].mContactPts.push_back(pt0);
					}

					if (h1.IsValid())
					{
						tContactPt pt1;
						const btVector3& contact_pt = pt.getPositionWorldOnB();
						pt1.mPos = tVector(contact_pt[0], contact_pt[1], contact_pt[2], 0) / world_scale;
						pt1.mForce = -force0;
						mContactEntries[h1.mID].mContactPts.push_back(pt1);
					}
				}
			}
		}
	}
}

cContactManager::tContactHandle cContactManager::RegisterContact(int contact_flags, int filter_flags)
{
	tContactHandle handle;
	handle.mFlags = contact_flags;
	handle.mFilterFlags = filter_flags;
	handle.mID = RegisterNewID();

	tContactEntry& entry = mContactEntries[handle.mID];
	entry.mFlags = contact_flags;
	entry.mFilterFlags = filter_flags;

	assert(handle.IsValid());
	return handle;
}

void cContactManager::UpdateContact(const cContactManager::tContactHandle& handle)
{
	assert(handle.IsValid());
	tContactEntry& entry = mContactEntries[handle.mID];
	entry.mFlags = handle.mFlags;
	entry.mFilterFlags = handle.mFilterFlags;
}

int cContactManager::GetNumEntries() const
{
	return static_cast<int>(mContactEntries.size());
}

bool cContactManager::IsInContact(const tContactHandle& handle) const
{
	if (handle.IsValid())
	{
		return mContactEntries[handle.mID].mContactPts.size() > 0;
	}
	return false;
}

const tEigenArr<cContactManager::tContactPt>& cContactManager::GetContactPts(const tContactHandle& handle) const
{
	return GetContactPts(handle.mID);
}

const tEigenArr<cContactManager::tContactPt>& cContactManager::GetContactPts(int handle_id) const
{
	assert(handle_id != gInvalidID);
	return mContactEntries[handle_id].mContactPts;
}

int cContactManager::RegisterNewID()
{
	int id = gInvalidID;
	id = static_cast<int>(mContactEntries.size());
	mContactEntries.resize(id + 1);
	return id;
}

void cContactManager::ClearContacts()
{
	int num_entries = GetNumEntries();
	for (int i = 0; i < num_entries; ++i)
	{
		tContactEntry& curr_entry = mContactEntries[i];
		curr_entry.mContactPts.clear();
	}
}

bool cContactManager::IsValidContact(const tContactHandle& h0, const tContactHandle& h1) const
{
	bool valid_h0 = ((h0.mFilterFlags & h1.mFlags) != 0);
	bool valid_h1 = ((h1.mFilterFlags & h0.mFlags) != 0);
	bool valid_contact = valid_h0 && valid_h1;
	return valid_contact;
}
