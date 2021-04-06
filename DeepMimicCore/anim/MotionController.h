#pragma once

#include "anim/KinController.h"
#include "anim/Motion.h"

class cMotionController : public cKinController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cMotionController();
	virtual ~cMotionController();

	virtual void Init(cKinCharacter* character, const std::string& param_file);
	virtual void Clear();

	virtual void CalcPose(double time, Eigen::VectorXd& out_pose) const;
	virtual void CalcVel(double time, Eigen::VectorXd& out_vel) const;
	
	virtual const cMotion& GetMotion() const;
	virtual cMotion& GetMotion();
	virtual int GetCycle() const;
	virtual double GetPhase() const;
	virtual void ChangeMotionDuration(double dur);

	virtual tVector GetCycleRootDelta() const;

protected:
	cMotion mMotion;
	tVector mCycleRootDelta;

	virtual bool LoadParams(const std::string& motion_file);
	virtual void CalcRootCycleOffset(double time, tVector& out_offset) const;
};