#pragma once

#include <memory>
#include "util/MathUtil.h"
#include "anim/Motion.h"

class cKinCharacter;

class cKinController : public std::enable_shared_from_this<cKinController>
{
public:
	virtual ~cKinController();

	virtual void Init(cKinCharacter* character, const std::string& param_file);
	virtual void Reset();
	virtual void Clear();
	virtual void Update(double time_step);

	virtual void CalcPose(double time, Eigen::VectorXd& out_pose) const = 0;
	virtual void CalcVel(double time, Eigen::VectorXd& out_vel) const = 0;

	virtual void SetTime(double time);
	virtual double GetTime() const;

	virtual const cMotion& GetMotion() const = 0;
	virtual cMotion& GetMotion() = 0;
	virtual double GetMotionDuration() const;
	virtual int GetNumMotionFrames() const;
	virtual cMotion::tFrame GetMotionFrame(int f) const;
	virtual cMotion::tFrame GetMotionFrameVel(int f) const;
	virtual bool EnableMotionLoop() const;
	virtual bool IsMotionOver() const;
	virtual void ChangeMotionDuration(double dur) = 0;

	virtual int GetCycle() const = 0;
	virtual double GetPhase() const = 0;

protected:
	cKinCharacter* mChar;
	double mTime;
	
	cKinController();

	virtual void ResetParams();
	virtual void BuildMotionParams(const std::string& motion_file, cKinCharacter& kin_char,
									cMotion::tParams& out_params) const;
	virtual bool LoadMotion(const std::string& motion_file, cMotion& out_motion) const;
	virtual void PostProcessMotion(cMotion& out_motion) const;
	virtual tVector CalcCycleRootDelta(const cMotion& motion) const;
};