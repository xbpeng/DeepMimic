#pragma once

#include "BulletDynamics/Featherstone/btMultiBodyConstraint.h"
#include "SimObj.h"
#include "SpAlg.h"

// for now joints are assumed to be hinge joints fixed along the z axis
class cSimJoint
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	struct tParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		tParams();

		int mID;
		cKinTree::eJointType mType;
		tVector mLimLow;
		tVector mLimHigh;
		double mTorqueLimit;
		double mForceLimit;
		bool mEnableAdjacentCollision;

		tVector mParentPos; // parent coords
		tVector mChildPos; // child coords
		tQuaternion mParentRot; // parent coords
		tQuaternion mChildRot; // child coords
	};

	cSimJoint();
	virtual ~cSimJoint();

	virtual void Init(std::shared_ptr<cWorld>& world, const std::shared_ptr<cSimObj>& parent_link, 
					const std::shared_ptr<cSimObj>& child_link, const tParams& params);
	virtual bool IsValid() const;

	virtual int GetID() const;
	virtual void SetID(int id);
	virtual cKinTree::eJointType GetType() const;

	virtual void CalcWorldRotation(tVector& out_axis, double& out_theta) const;
	virtual tQuaternion CalcWorldRotation() const;
	virtual tMatrix BuildWorldTrans() const;

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

	virtual const tVector& GetParentPos() const; // in parent link's coordinates
	virtual const tVector& GetChildPos() const; // in child link's coordinates
	virtual const tQuaternion& GetParentRot() const; // in parent link's coordinates
	virtual const tQuaternion& GetChildRot() const; // in child link's coordinates
	virtual tMatrix BuildJointChildTrans() const;
	virtual tMatrix BuildJointParentTrans() const;
	virtual tVector CalcAxisWorld() const;
	virtual tVector GetAxisRel() const;
	
	virtual bool HasParent() const;
	virtual bool HasChild() const;

	virtual const std::shared_ptr<cSimObj>& GetParent() const;
	virtual const std::shared_ptr<cSimObj>& GetChild() const;

	virtual void ClampTotalTorque(tVector& out_torque) const;
	virtual void ClampTotalForce(tVector& out_force) const;

	virtual const tVector& GetLimLow() const;
	virtual const tVector& GetLimHigh() const;
	virtual bool HasJointLim() const;
	virtual bool EnableAdjacentCollision() const;

	virtual int GetParamSize() const;
	virtual void BuildPose(Eigen::VectorXd& out_pose) const;
	virtual void BuildVel(Eigen::VectorXd& out_vel) const;
	virtual bool HasMultBody() const;

	virtual void Bind();
	virtual void Unbind();

	virtual tVector GetTotalTorque() const;
	virtual tVector GetTotalForce() const;

	virtual const std::shared_ptr<btTypedConstraint>& GetCons() const;
	virtual const std::shared_ptr<btMultiBodyConstraint>& GetMultBodyCons() const;

protected:
	tParams mParams;
	std::shared_ptr<cWorld> mWorld;
	std::shared_ptr<cSimObj> mParent;
	std::shared_ptr<cSimObj> mChild;

	// all torques and forces are in local coordinates
	cSpAlg::tSpVec mTotalTau;
	tQuaternion mJoint2ParentQuat;
	tQuaternion mJoint2ChildQuat;

	std::shared_ptr<btTypedConstraint> mCons;
	std::shared_ptr<btMultiBodyConstraint> mMultBodyCons;

	virtual void RemoveFromWorld();

	virtual tVector CalcParentLocalPos(const tVector& local_pos) const;
	virtual tVector CalcChildLocalPos(const tVector& local_pos) const;
	virtual double GetPrismaticOffset() const;

	virtual void BuildConstraint(std::shared_ptr<cWorld>& world);
	virtual void BuildConstraintRevolute(std::shared_ptr<cWorld>& world);
	virtual void BuildConstraintPlanar(std::shared_ptr<cWorld>& world);
	virtual void BuildConstraintPrismatic(std::shared_ptr<cWorld>& world);
	virtual void BuildConstraintFixed(std::shared_ptr<cWorld>& world);
	virtual void BuildConstraintSpherical(std::shared_ptr<cWorld>& world);

	virtual void BuildPoseRevolute(Eigen::VectorXd& out_pose) const;
	virtual void BuildPosePlanar(Eigen::VectorXd& out_pose) const;
	virtual void BuildPosePristmatic(Eigen::VectorXd& out_pose) const;
	virtual void BuildPoseFixed(Eigen::VectorXd& out_pose) const;
	virtual void BuildPoseSpherical(Eigen::VectorXd& out_pose) const;

	virtual void BuildVelRevolute(Eigen::VectorXd& out_pose) const;
	virtual void BuildVelPlanar(Eigen::VectorXd& out_pose) const;
	virtual void BuildVelPristmatic(Eigen::VectorXd& out_pose) const;
	virtual void BuildVelFixed(Eigen::VectorXd& out_pose) const;
	virtual void BuildVelSpherical(Eigen::VectorXd& out_vel) const;

	virtual void SetTotalTorque(const tVector& torque);
	virtual void SetTotalForce(const tVector& force);
	virtual void ApplyTorque();
	virtual void ApplyForce();

	virtual tVector CalcLocalAngVel() const;
	virtual tVector CalcLocalLinVel() const;
};
