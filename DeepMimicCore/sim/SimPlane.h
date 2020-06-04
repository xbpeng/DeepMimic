#pragma once

#include "sim/SimRigidBody.h"

class cSimPlane : public cSimRigidBody
{
public:
	struct tParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		tParams();

		double mMass;
		double mFriction;
		tVector mOrigin;
		tVector mNormal;
	};

	cSimPlane();
	virtual ~cSimPlane();

	virtual void Init(const std::shared_ptr<cWorld>& world, const tParams& params);
	virtual tVector GetCoeffs() const;
	virtual cShape::eShape GetShape() const;
	virtual tVector GetSize() const override;

protected:
};
