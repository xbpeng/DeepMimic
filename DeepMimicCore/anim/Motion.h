#pragma once

#include "util/json/json.h"
#include "KinTree.h"

class cMotion
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	typedef Eigen::VectorXd tFrame;
	typedef std::function<void(const tFrame* a, const tFrame* b, double lerp, tFrame* out_frame)> tBlendFunc;
	typedef std::function<void(const tFrame* a, const tFrame* b, double timestep, tFrame* out_vel)> tVelFunc;
	typedef std::function<void(tFrame* out_frame)> tPostProcessFunc;

	enum eLoop
	{
		eLoopNone,
		eLoopWrap,
		eLoopMax
	};

	struct tParams
	{
		std::string mMotionFile;
		tBlendFunc mBlendFunc;
		tVelFunc mVelFunc;
		tPostProcessFunc mPostProcessFunc;

		tParams();
	};

	static std::string BuildFrameJson(const Eigen::VectorXd& frame);

	static double CalcPhase(double time, double period, double phase_offset, bool loop);
	static double CalcFixPhaseTimeOffset(double time, double prev_period, double new_period,
										double time_offset, double phase_offset, bool loop);

	cMotion();
	virtual ~cMotion();

	virtual void Clear();
	virtual bool Load(const tParams& params);
	virtual void Init(int num_frames, int num_dofs);
	virtual bool IsValid() const;

	virtual int GetNumFrames() const;
	virtual int GetNumDof() const;
	virtual tFrame GetFrame(int f) const;
	virtual tFrame GetFrameVel(int f) const;
	virtual void SetFrame(int f, const tFrame& frame);
	virtual void SetFrameTime(int f, double time);
	virtual void BlendFrames(int a, int b, double lerp, tFrame& out_frame) const;

	virtual void CalcFrame(double time, tFrame& out_frame) const;
	virtual void CalcFrameVel(double time, cMotion::tFrame& out_vel) const;
	virtual void CalcFramePhase(double phase, tFrame& out_frame) const;
	virtual double GetDuration() const;
	virtual void ChangeDuration(double dur);
	virtual double GetFrameTime(int f) const;
	virtual double GetFrameDuration(int f) const;

	virtual double CalcPhase(double time) const;
	virtual int CalcCycleCount(double time) const;
	virtual void UpdateVel();

	virtual void TimeWrap(double scale);

	virtual bool IsOver(double time) const;

	virtual bool HasBlendFunc() const;
	virtual bool HasVelFunc() const;
	virtual bool HasPostProcessFunc() const;
	virtual bool EnableLoop() const;
	virtual eLoop GetLoop() const;
	virtual void SetLoop(eLoop loop);

	virtual void Output(const std::string& out_filepath) const;

protected:
	eLoop mLoop;
	bool mCycleSyncRootPos;
	bool mCycleSyncRootRot;
	bool mCycleSyncRootHeight;

	tParams mParams;
	Eigen::VectorXd mFrameTimes;
	Eigen::MatrixXd mFrames;
	Eigen::MatrixXd mFrameVel;

	tVector mCycleDeltaRootPos;
	double mCycleDeltaRootHeading;

	virtual bool LoadJson(const Json::Value& root);
	virtual bool ParseLoop(const std::string& str, eLoop& out_loop) const;
	virtual bool LoadJsonFrames(const Json::Value& root, Eigen::VectorXd& out_frame_times, Eigen::MatrixXd& out_frames) const;
	virtual bool ParseFrameJson(const Json::Value& root, Eigen::VectorXd& out_frame) const;
	virtual std::string BuildLoopStr(eLoop loop) const;

	virtual void PostProcessFrames(Eigen::VectorXd& frame_times, Eigen::MatrixXd& frames) const;
	virtual int GetFrameSize() const;
	virtual void BuildFrameVel(Eigen::MatrixXd& out_frame_vel) const;
	virtual tVector CalcCycleDeltaRootPos() const;
	virtual double CalcCycleDeltaRootHeading() const;

	virtual void CalcIndexBlend(double time, int& out_idx, double& out_blend) const;

	virtual void BlendFramesIntern(const cMotion::tFrame* a, const cMotion::tFrame* b, double lerp, cMotion::tFrame* out_frame) const;
	virtual void CalcFrameVel(const tFrame& frame0, const tFrame& frame1, double dt, tFrame& out_vel) const;
};
