#pragma once

#include "sim/SimRigidBody.h"

class cSimSphere : public cSimRigidBody
{
public:
	struct tParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		tParams();

		eType mType;
		double mMass;
		double mFriction;
		tVector mPos;
		tVector mVel;
		double mRadius;
		tVector mAxis;
		double mTheta;
	};

	cSimSphere();
	virtual ~cSimSphere();

	virtual void Init(const std::shared_ptr<cWorld>& world, const tParams& params);
	virtual double GetRadius() const;
	virtual tVector GetSize() const override;

	virtual cShape::eShape GetShape() const override;

protected:
};
