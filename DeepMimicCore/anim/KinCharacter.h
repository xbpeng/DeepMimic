#pragma once

#include "anim/Character.h"
#include "anim/Motion.h"
#include "anim/KinController.h"

class cKinCharacter : public cCharacter
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
		tVector mOrigin;
		bool mLoadDrawShapes;
	};

	cKinCharacter();
	virtual ~cKinCharacter();

	virtual bool Init(const tParams& params);
	virtual void Clear();
	virtual void Update(double time_step);
	virtual void Reset();

	virtual bool LoadMotion(const std::string& motion_file);
	virtual const cMotion* GetMotion() const;
	virtual cMotion* GetMotion();
	virtual double GetMotionDuration() const;
	virtual void ChangeMotionDuration(double dur);
	virtual bool EnableMotionLoop() const;
	virtual int GetNumMotionFrames() const;

	virtual void SetTime(double time);
	virtual double GetTime() const;
	virtual int GetCycle() const;
	virtual double GetPhase() const;

	virtual void Pose();
	virtual void Pose(double time);
	virtual void BuildAcc(Eigen::VectorXd& out_acc) const;
	virtual cMotion::tFrame GetMotionFrame(int f) const;
	virtual cMotion::tFrame GetMotionFrameVel(int f) const;

	virtual void SetRootPos(const tVector& pos);
	virtual void SetRootRotation(const tQuaternion& q);

	virtual const tVector& GetOriginPos() const;
	virtual void SetOriginPos(const tVector& origin);
	virtual void MoveOrigin(const tVector& delta);
	virtual const tQuaternion& GetOriginRot() const;
	virtual void SetOriginRot(const tQuaternion& rot);
	virtual void RotateOrigin(const tQuaternion& rot);

	virtual void CalcPose(double time, Eigen::VectorXd& out_pose) const;
	virtual void CalcVel(double time, Eigen::VectorXd& out_vel) const;
	virtual void CalcAcc(double time, Eigen::VectorXd& out_vel) const;

	virtual bool IsMotionOver() const;

	// motion processing methods
	virtual void BlendFrames(const cMotion::tFrame* a, const cMotion::tFrame* b, double lerp, cMotion::tFrame* out_frame) const;
	virtual void BlendVel(const Eigen::VectorXd* a, const Eigen::VectorXd* b, double lerp, Eigen::VectorXd* out_vel) const;
	virtual void CalcFrameVel(const cMotion::tFrame* a, const cMotion::tFrame* b, double timestep, cMotion::tFrame* out_vel) const;
	virtual void PostProcessPose(cMotion::tFrame* out_frame) const;
	
	virtual void SetController(const std::shared_ptr<cKinController>& ctrl);
	virtual const std::shared_ptr<cKinController>& GetController() const;
	virtual void RemoveController();
	virtual bool HasController() const;

	virtual tVector GetCycleRootDelta() const;

protected:
	std::shared_ptr<cKinController> mController;

	tVector mOrigin;
	tQuaternion mOriginRot;
};