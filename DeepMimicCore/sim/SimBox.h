#pragma once

#include "sim/SimRigidBody.h"

class cSimBox : public cSimRigidBody
{
public:
	struct tParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		eType mType;
		double mMass;
		double mFriction;
		tVector mPos;
		tVector mVel;
		tVector mSize;
		tVector mAxis;
		double mTheta;

		tParams();
	};

	cSimBox();
	virtual ~cSimBox();

	virtual void Init(const std::shared_ptr<cWorld>& world, const tParams& params);
	virtual tVector GetSize() const;

	virtual cShape::eShape GetShape() const override;

protected:
};