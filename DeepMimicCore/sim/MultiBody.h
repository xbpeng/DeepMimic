#pragma once
#include "BulletDynamics/Featherstone/btMultiBody.h"

class cMultiBody : public btMultiBody
{
public:
	cMultiBody(int n_links,             // NOT including the base
		btScalar mass,                // mass of base
		const btVector3 &inertia,    // inertia of base, in base frame; assumed diagonal
		bool fixedBase,           // whether the base is fixed (true) or can move (false)
		bool canSleep, bool deprecatedMultiDof=true);

	virtual ~cMultiBody();

	void compTreeLinkVelocities(btVector3 *omega, btVector3 *vel) const;
	
	void setupPlanar(int i,
					btScalar mass,
					const btVector3 &inertia,
					int parent,
					const btQuaternion &rotParentToThis,
					const btVector3 &rotationAxis,
					const btVector3 &parentComToThisComOffset,
					bool disableParentCollision);


protected:

	static void SpatialTransform(const btMatrix3x3 &rotation_matrix,  // rotates vectors in 'from' frame to vectors in 'to' frame
								const btVector3 &displacement,     // vector from origin of 'from' frame to origin of 'to' frame, in 'to' coordinates
								const btVector3 &top_in,       // top part of input vector
								const btVector3 &bottom_in,    // bottom part of input vector
								btVector3 &top_out,         // top part of output vector
								btVector3 &bottom_out);      // bottom part of output vector

};
