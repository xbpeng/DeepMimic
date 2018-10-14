#pragma once

#include <memory>
#include "util/MathUtil.h"

class cWorld;

class cContactManager
{
public:
	const static int gInvalidID;
	const static short gFlagAll = -1;
	const static short gFlagNone = 0;
	const static short gFlagRayTest = 1;

	struct tContactHandle
	{
		int mID;
		int mFlags;
		int mFilterFlags;

		tContactHandle();
		bool IsValid() const;
	};

	struct tContactPt
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		tContactPt();

		tVector mPos;
		tVector mForce;
	};

	cContactManager(cWorld& world);
	virtual ~cContactManager();

	virtual void Init();
	virtual void Reset();
	virtual void Clear();
	virtual void Update();

	virtual tContactHandle RegisterContact(int contact_flags, int filter_flags);
	virtual void UpdateContact(const cContactManager::tContactHandle& handle);
	virtual int GetNumEntries() const;
	virtual bool IsInContact(const tContactHandle& handle) const;
	virtual const tEigenArr<tContactPt>& GetContactPts(const tContactHandle& handle) const;
	virtual const tEigenArr<tContactPt>& GetContactPts(int handle_id) const;

protected:
	struct tContactEntry
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		tContactEntry();

		int mFlags;
		int mFilterFlags;
		tEigenArr<tContactPt> mContactPts;
	};

	cWorld& mWorld;
	tEigenArr<tContactEntry> mContactEntries;

	virtual int RegisterNewID();
	virtual void ClearContacts();
	virtual bool IsValidContact(const tContactHandle& h0, const tContactHandle& h1) const;
};