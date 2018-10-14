#pragma once

#include "util/json/json.h"
#include "KinTree.h"

class cMotion
{
public:
	enum eFrameParams
	{
		eFrameTime,
		eFrameMax
	};
	typedef Eigen::VectorXd tFrame;
	typedef std::function<void(const tFrame* a, const tFrame* b, double lerp, tFrame* out_frame)> tBlendFunc;
	typedef std::function<void(const std::vector<int>* right_joints, const std::vector<int>* left_joints, tFrame* out_frame)> tMirrorFunc;
	typedef std::function<void(const tFrame* a, const tFrame* b, double timestep, tFrame* out_vel)> tVelFunc;
	typedef std::function<void(tFrame* out_frame)> tPostProcessFunc;

	enum eLoop
	{
		eLoopNone,
		eLoopWrap,
		eLoopMirror,
		eLoopMax
	};

	struct tParams
	{
		std::string mMotionFile;
		tBlendFunc mBlendFunc;
		tMirrorFunc mMirrorFunc;
		tVelFunc mVelFunc;
		tPostProcessFunc mPostProcessFunc;

		std::vector<int> mRightJoints;
		std::vector<int> mLeftJoints;

		tParams();
	};

	static const std::string gFrameKey;
	static const std::string gLoopKey;
	static const std::string gVelFilterCutoffKey;
	static std::string BuildFrameJson(const Eigen::VectorXd& frame);

	cMotion();
	virtual ~cMotion();

	virtual void Clear();
	virtual bool Load(const tParams& params);
	virtual void Init(int num_frames, int num_dofs);
	virtual bool IsValid() const;

	virtual int GetNumFrames() const;
	virtual int GetNumDof() const;
	virtual tFrame GetFrame(int i) const;
	virtual void SetFrame(int i, const tFrame& frame);
	virtual void SetFrameTime(int i, double time);
	virtual void BlendFrames(int a, int b, double lerp, tFrame& out_frame) const;

	virtual void CalcFrame(double time, tFrame& out_frame, bool force_mirror = false) const;
	virtual void CalcFrameVel(double time, cMotion::tFrame& out_vel, bool force_mirror = false) const;
	virtual void CalcFramePhase(double phase, tFrame& out_frame, bool force_mirror = false) const;
	virtual double GetDuration() const;
	virtual void SetDuration(double dur);
	virtual double GetFrameTime(int f) const;
	virtual double GetFrameDuration(int f) const;

	virtual int CalcCycleCount(double time) const;
	virtual void CalcIndexPhase(double time, int& out_idx, double& out_phase) const;
	virtual void UpdateVel();

	virtual bool IsOver(double time) const;

	virtual bool HasBlendFunc() const;
	virtual bool HasMirrorFunc() const;
	virtual bool HasVelFunc() const;
	virtual bool HasPostProcessFunc() const;
	virtual bool EnableLoop() const;
	virtual eLoop GetLoop() const;
	virtual void SetLoop(eLoop loop);

	virtual void Output(const std::string& out_filepath) const;

protected:
	eLoop mLoop;
	double mVelFilterCutoff;
	tParams mParams;
	Eigen::MatrixXd mFrames;
	Eigen::MatrixXd mFrameVel;
	Eigen::MatrixXd mFrameVelMirror;

	virtual bool LoadJson(const Json::Value& root);
	virtual bool ParseLoop(const std::string& str, eLoop& out_loop) const;
	virtual bool LoadJsonFrames(const Json::Value& root, Eigen::MatrixXd& out_frames) const;
	virtual bool ParseFrameJson(const Json::Value& root, Eigen::VectorXd& out_frame) const;
	virtual bool LoadJsonJoints(const Json::Value& root, std::vector<int>& out_right_joints, std::vector<int>& out_left_joints) const;
	virtual std::string BuildLoopStr(eLoop loop) const;

	virtual void PostProcessFrames(Eigen::MatrixXd& frames) const;
	virtual int GetFrameSize() const;
	virtual void BuildFrameVel(Eigen::MatrixXd& out_frame_vel, bool mirror = false) const;
	virtual void FilterFrameVel(Eigen::MatrixXd& out_frame_vel) const;

	virtual void BlendFramesIntern(const cMotion::tFrame* a, const cMotion::tFrame* b, double lerp, cMotion::tFrame* out_frame) const;
	virtual bool NeedMirrorFrame(double time) const;
	virtual void MirrorFrame(tFrame& out_frame) const;
	virtual void CalcFrameVel(const tFrame& frame0, const tFrame& frame1, double dt, tFrame& out_vel) const;
};
