#include "Perturb.h"
#include "sim/SimObj.h"

tPerturb tPerturb::BuildForce()
{
	tPerturb perturb;
	perturb.mType = ePerturbForce;
	return perturb;
}

tPerturb::tPerturb()
{
	Clear();
}

tPerturb::tPerturb(ePerturb type, cSimObj* obj, const tVector& local_pos,
					const tVector& perturb, double duration)
	: tPerturb()
{
	assert(type != ePerturbInvalid);
	mType = type;
	mObj = obj;
	mLocalPos = local_pos;
	mPerturb = perturb;
	mDuration = duration;
}

tPerturb::~tPerturb()
{
}

bool tPerturb::HasExpired() const
{
	return mTime >= mDuration;
}

void tPerturb::Clear()
{
	mType = ePerturbInvalid;
	mObj = nullptr;
	mLocalPos.setZero();
	mPerturb.setZero();
	mDuration = 0;
	mTime = 0;
}

bool tPerturb::IsValid() const
{
	return (mObj != nullptr) && (mType != ePerturbInvalid);
}

void tPerturb::Update(double time_step)
{
	mTime += time_step;
	switch (mType)
	{
	case ePerturbForce:
		ApplyForce();
		break;
	case ePerturbTorque:
		ApplyTorque();
		break;
	default:
		assert(false); // unsupported perturb type
		break;
	}
}

void tPerturb::ApplyForce()
{
	assert(mType == ePerturbForce);
	mObj->ApplyForce(mPerturb, mLocalPos);
}

void tPerturb::ApplyTorque()
{
	assert(mType == ePerturbTorque);
	mObj->ApplyTorque(mPerturb);
}
