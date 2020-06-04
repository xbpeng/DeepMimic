#pragma once

#include "sim/World.h"
#include "sim/SimBodyLink.h"
#include "sim/SimBodyJoint.h"
#include "sim/CharController.h"
#include "sim/SimObj.h"
#include "anim/Character.h"
#include "MultiBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h"

class cSimCharacter : public virtual cCharacter, public virtual cSimObj
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	struct tParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		tParams();

		int mID;
		std::string mCharFile;
		std::string mStateFile;
		tVector mInitPos;
		bool mLoadDrawShapes;

		bool mEnableContactFall;
	};

	cSimCharacter();
	virtual ~cSimCharacter();

	virtual bool Init(const std::shared_ptr<cWorld>& world, const tParams& params);
	virtual void Clear();
	virtual void Reset();
	virtual void Update(double time_step);
	virtual void PostUpdate(double time_step);

	virtual tVector GetRootPos() const;
	virtual void GetRootRotation(tVector& out_axis, double& out_theta) const;
	virtual tQuaternion GetRootRotation() const;
	virtual tVector GetRootVel() const;
	virtual tVector GetRootAngVel() const;
	virtual const Eigen::MatrixXd& GetBodyDefs() const;
	virtual void SetRootPos(const tVector& pos);
	virtual void SetRootRotation(const tVector& axis, double theta);
	virtual void SetRootRotation(const tQuaternion& q);
	virtual void SetRootTransform(const tVector& pos, const tQuaternion& rot);

	virtual void SetRootVel(const tVector& vel);
	virtual void SetRootAngVel(const tVector& ang_vel);
	
	virtual tQuaternion CalcHeadingRot() const;

	virtual int GetNumBodyParts() const;

	virtual void SetPose(const Eigen::VectorXd& pose);
	virtual void SetVel(const Eigen::VectorXd& vel);

	virtual tVector CalcJointPos(int joint_id) const;
	virtual tVector CalcJointVel(int joint_id) const;
	virtual void CalcJointWorldRotation(int joint_id, tVector& out_axis, double& out_theta) const;
	virtual tQuaternion CalcJointWorldRotation(int joint_id) const;
	virtual tMatrix BuildJointWorldTrans(int joint_id) const;

	virtual tVector CalcCOM() const;
	virtual tVector CalcCOMVel() const;
	virtual void CalcAABB(tVector& out_min, tVector& out_max) const;
	virtual tVector GetSize() const override;

	virtual const cSimBodyJoint& GetJoint(int joint_id) const;
	virtual cSimBodyJoint& GetJoint(int joint_id);
	virtual const std::shared_ptr<cSimBodyLink>& GetBodyPart(int idx) const;
	virtual std::shared_ptr<cSimBodyLink>& GetBodyPart(int idx);
	virtual tVector GetBodyPartPos(int idx) const;
	virtual tVector GetBodyPartVel(int idx) const;
	virtual const std::shared_ptr<cSimBodyLink>& GetRootPart() const;
	virtual std::shared_ptr<cSimBodyLink> GetRootPart();

	virtual void RegisterContacts(int contact_flags, int filter_flags);
	virtual void UpdateContact(int contact_flags, int filter_flags);
	virtual bool IsInContact() const;
	virtual bool IsInContact(int idx) const;
	virtual const tEigenArr<cContactManager::tContactPt>& GetContactPts(int idx) const;
	virtual const tEigenArr<cContactManager::tContactPt>& GetContactPts() const;
	virtual const cContactManager::tContactHandle& GetContactHandle() const;
	
	virtual bool HasFallen() const;
	virtual bool HasStumbled() const;
	virtual bool HasVelExploded(double vel_threshold = 100.0) const;

	virtual bool IsValidBodyPart(int idx) const;
	virtual bool EnableBodyPartFallContact(int idx) const;
	virtual void SetBodyPartFallContact(int idx, bool enable);

	virtual void SetController(std::shared_ptr<cCharController> ctrl);
	virtual void RemoveController();
	virtual bool HasController() const;
	virtual const std::shared_ptr<cCharController>& GetController();
	virtual const std::shared_ptr<cCharController>& GetController() const;
	virtual void EnableController(bool enable);

	virtual void ApplyForce(const tVector& force);
	virtual void ApplyForce(const tVector& force, const tVector& local_pos);
	virtual void ApplyTorque(const tVector& torque);
	virtual void ClearForces();
	virtual void ApplyControlForces(const Eigen::VectorXd& tau);
	virtual void PlayPossum();

	virtual tVector GetPartColor(int part_id) const;
	virtual double CalcTotalMass() const;

	virtual void SetLinearDamping(double damping);
	virtual void SetAngularDamping(double damping);

	virtual const std::shared_ptr<cWorld>& GetWorld() const;
	virtual const std::shared_ptr<cMultiBody>& GetMultiBody() const;
	virtual const std::vector<std::shared_ptr<btMultiBodyJointLimitConstraint>>& GetConstraints() const;

	// cSimObj Interface
	virtual tVector GetPos() const;
	virtual void SetPos(const tVector& pos);
	virtual void GetRotation(tVector& out_axis, double& out_theta) const;
	virtual tQuaternion GetRotation() const;
	virtual void SetRotation(const tVector& axis, double theta);
	virtual void SetRotation(const tQuaternion& q);
	virtual tMatrix GetWorldTransform() const;

	virtual tVector GetLinearVelocity() const;
	virtual tVector GetLinearVelocity(const tVector& local_pos) const;
	virtual void SetLinearVelocity(const tVector& vel);
	virtual tVector GetAngularVelocity() const;
	virtual void SetAngularVelocity(const tVector& vel);

	virtual short GetColGroup() const;
	virtual void SetColGroup(short col_group);
	virtual short GetColMask() const;
	virtual void SetColMask(short col_mask);

protected:
	std::shared_ptr<cMultiBody> mMultBody;
	std::vector<std::shared_ptr<cSimBodyLink>> mBodyParts;
	tEigenArr<cSimBodyJoint> mJoints;
	std::vector<std::shared_ptr<btMultiBodyJointLimitConstraint>> mCons;

	double mFriction;
	Eigen::MatrixXd mBodyDefs;

	std::shared_ptr<cCharController> mController;
	tQuaternion mInvRootAttachRot;

	// scratch buffers for intermediate computation
	btAlignedObjectArray<btVector3> mVecBuffer0;
	btAlignedObjectArray<btVector3> mVecBuffer1;
	btAlignedObjectArray<btQuaternion> mRotBuffer;

	virtual bool LoadBodyDefs(const std::string& char_file, Eigen::MatrixXd& out_body_defs) const;
	
	virtual bool BuildSimBody(const tParams& params);
	virtual bool BuildMultiBody(std::shared_ptr<cMultiBody>& out_body);
	virtual bool BuildConstraints(std::shared_ptr<cMultiBody>& out_body);
	
	virtual bool BuildBodyLinks();
	virtual btCollisionShape* BuildCollisionShape(const cShape::eShape shape, const tVector& shape_size);
	
	virtual bool BuildJoints();
	virtual void BuildConsFactor(int joint_id, tVector& out_linear_factor, tVector& out_angular_factor) const;
	virtual void BuildRootConsFactor(cKinTree::eJointType joint_type, tVector& out_linear_factor, tVector& out_angular_factor) const;
	virtual bool FixedBase() const;
	virtual void RemoveFromWorld();

	virtual void ClearJointTorques();
	virtual void UpdateJoints();
	virtual void UpdateLinkPos();
	virtual void UpdateLinkVel();

	virtual short GetPartColGroup(int part_id) const;
	virtual short GetPartColMask(int part_id) const;

	virtual void BuildJointPose(int joint_id, Eigen::VectorXd& out_pose) const;
	virtual void BuildJointVel(int joint_id, Eigen::VectorXd& out_vel) const;

	virtual void BuildPose(Eigen::VectorXd& out_pose) const;
	virtual void BuildVel(Eigen::VectorXd& out_vel) const;

	virtual bool CheckFallContact() const;
	virtual const btCollisionObject* GetCollisionObject() const;
	virtual btCollisionObject* GetCollisionObject();
};
