#pragma once

#include <memory>
#include "KinTree.h"
#include "render/DrawMesh.h"

class cCharacter
{
public:
	static const std::string gSkeletonKey;

	virtual ~cCharacter();

	virtual void SetID(int id);
	virtual int GetID() const;

	virtual bool Init(const std::string& char_file, bool load_draw_shapes);
	virtual void Clear();
	virtual void Update(double time_step);
	virtual void Reset();

	virtual int GetNumDof() const;
	virtual const Eigen::MatrixXd& GetJointMat() const;
	virtual int GetNumJoints() const;
	virtual cKinTree::eJointType GetJointType(int joint_id) const;

	virtual const Eigen::VectorXd& GetPose() const;
	virtual void SetPose(const Eigen::VectorXd& pose);
	virtual const Eigen::VectorXd& GetPose0() const;
	virtual void SetPose0(const Eigen::VectorXd& pose);

	virtual const Eigen::VectorXd& GetVel() const;
	virtual void SetVel(const Eigen::VectorXd& vel);
	virtual const Eigen::VectorXd& GetVel0() const;
	virtual void SetVel0(const Eigen::VectorXd& vel);

	virtual int GetRootID() const;
	virtual tVector GetRootPos() const;
	virtual void GetRootRotation(tVector& out_axis, double& out_theta) const;
	virtual tQuaternion GetRootRotation() const;
	virtual void SetRootPos(const tVector& pos);
	virtual void SetRootPos0(const tVector& pose);
	virtual void SetRootRotation(const tQuaternion& q);
	virtual void RotateRoot(const tQuaternion& rot);

	virtual void SetRootVel(const tVector& vel);
	virtual void SetRootAngVel(const tVector& ang_vel);

	virtual tQuaternion CalcHeadingRot() const;

	virtual double CalcHeading() const;
	virtual tMatrix BuildOriginTrans() const;

	virtual int GetParamOffset(int joint_id) const;
	virtual int GetParamSize(int joint_id) const;
	virtual bool IsEndEffector(int joint_id) const;
	virtual int GetParentJoint(int joint_id) const;

	virtual tVector CalcJointPos(int joint_id) const;
	virtual tVector CalcJointVel(int joint_id) const;
	virtual void CalcJointWorldRotation(int joint_id, tVector& out_axis, double& out_theta) const;
	virtual tQuaternion CalcJointWorldRotation(int joint_id) const;
	virtual double CalcJointChainLength(int joint_id);
	virtual tMatrix BuildJointWorldTrans(int joint_id) const;

	virtual void CalcAABB(tVector& out_min, tVector& out_max) const;
	virtual int GetNumEndEffectors() const;
	virtual const Eigen::VectorXi& GetEndEffectors() const;

	// weights for each joint used to compute the pose error during training
	virtual double GetJointDiffWeight(int joint_id) const;

	virtual bool WriteState(const std::string& file) const;
	virtual bool WriteState(const std::string& file, const tMatrix& root_trans) const;
	virtual bool ReadState(const std::string& file);

	virtual bool LoadSkeleton(const Json::Value& root);
	virtual void InitDefaultState();

	virtual bool HasDrawShapes() const;
	virtual const Eigen::MatrixXd& GetDrawShapeDefs() const;
	virtual const std::shared_ptr<cDrawMesh>& GetMesh(int i) const;
	virtual int GetNumMeshes() const;
	
protected:
	int mID;
	Eigen::MatrixXd mJointMat;
	Eigen::VectorXd mPose;
	Eigen::VectorXd mVel;
	Eigen::VectorXd mPose0;
	Eigen::VectorXd mVel0;

	Eigen::VectorXi mEndEffectors;

	Eigen::MatrixXd mDrawShapeDefs;
	std::vector<std::shared_ptr<cDrawMesh>> mMeshes;

	cCharacter();

	virtual void ResetParams();
	virtual bool ParseState(const Json::Value& root, Eigen::VectorXd& out_state) const;
	virtual std::string BuildStateJson(const Eigen::VectorXd& pose, const Eigen::VectorXd& vel) const;

	virtual bool LoadDrawShapeDefs(const std::string& char_file, Eigen::MatrixXd& out_draw_defs) const;
	virtual bool LoadMeshes(const std::string& char_file, std::vector<std::shared_ptr<cDrawMesh>>& out_meshes) const;

	virtual void RecordEndEffectors(Eigen::VectorXi& out_end_effs) const;
};