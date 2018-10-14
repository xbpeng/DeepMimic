#pragma once

#include "SimBodyLink.h"
#include "SpAlg.h"

// for now joints are assumed to be hinge joints fixed along the z axis
class cSimBodyJoint
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	struct tParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		tParams();

		int mID;
		tVector mLimLow;
		tVector mLimHigh;
		double mTorqueLimit;
		double mForceLimit;

		tVector mParentPos; // parent coords
		tVector mChildPos; // child coords
		tQuaternion mParentRot; // parent coords
		tQuaternion mChildRot; // child coords
	};

	cSimBodyJoint();
	virtual ~cSimBodyJoint();

	virtual void Init(std::shared_ptr<cWorld>& world, const std::shared_ptr<cMultiBody>& multbody,
					const std::shared_ptr<cSimBodyLink>& parent, const std::shared_ptr<cSimBodyLink>& child, 
					const tParams& params);
	virtual void Clear();
	virtual bool IsValid() const;

	virtual cKinTree::eJointType GetType() const;

	virtual void CalcWorldRotation(tVector& out_axis, double& out_theta) const;
	virtual tQuaternion CalcWorldRotation() const;
	virtual tMatrix BuildWorldTrans() const;
	virtual bool IsRoot() const;

	virtual void AddTau(const Eigen::VectorXd& tau);
	virtual const cSpAlg::tSpVec& GetTau() const;
	virtual void ApplyTau();
	virtual void ClearTau();

	virtual tVector CalcWorldPos() const;
	virtual tVector CalcWorldPos(const tVector& local_pos) const;
	virtual tVector CalcWorldVel() const;
	virtual tVector CalcWorldVel(const tVector& local_pos) const;
	virtual tVector CalcWorldAngVel() const;
	virtual double GetTorqueLimit() const;
	virtual double GetForceLimit() const;
	virtual void SetTorqueLimit(double lim);
	virtual void SetForceLimit(double lim);

	virtual tVector GetParentPos() const; // in parent link's coordinates
	virtual tVector GetChildPos() const; // in child link's coordinates
	virtual tMatrix BuildJointChildTrans() const;
	virtual tMatrix BuildJointParentTrans() const;
	virtual tVector CalcAxisWorld() const;
	virtual tVector GetAxisRel() const;

	virtual bool HasParent() const;
	virtual bool HasChild() const;

	virtual const std::shared_ptr<cSimBodyLink>& GetParent() const;
	virtual const std::shared_ptr<cSimBodyLink>& GetChild() const;

	virtual void ClampTotalTorque(tVector& out_torque) const;
	virtual void ClampTotalForce(tVector& out_force) const;
	
	virtual const tVector& GetLimLow() const;
	virtual const tVector& GetLimHigh() const;
	virtual bool HasJointLim() const;

	virtual int GetParamSize() const;
	virtual void BuildPose(Eigen::VectorXd& out_pose) const;
	virtual void BuildVel(Eigen::VectorXd& out_vel) const;
	virtual void SetPose(const Eigen::VectorXd& pose);
	virtual void SetVel(const Eigen::VectorXd& vel);

	virtual tVector GetTotalTorque() const;
	virtual tVector GetTotalForce() const;

protected:
	cKinTree::eJointType mType;
	tParams mParams;
	std::shared_ptr<cWorld> mWorld;
	std::shared_ptr<cMultiBody> mMultBody;
	std::shared_ptr<cSimBodyLink> mParent;
	std::shared_ptr<cSimBodyLink> mChild;

	// all torques and forces are in local coordinates
	cSpAlg::tSpVec mTotalTau;

	virtual cKinTree::eJointType FetchJointType() const;

	virtual tVector CalcParentLocalPos(const tVector& local_pos) const;
	virtual tVector CalcChildLocalPos(const tVector& local_pos) const;
	
	virtual void SetTotalTorque(const tVector& torque);
	virtual void SetTotalForce(const tVector& force);
	virtual void ApplyTauRevolute();
	virtual void ApplyTauPlanar();
	virtual void ApplyTauPrismatic();
	virtual void ApplyTauSpherical();
};
