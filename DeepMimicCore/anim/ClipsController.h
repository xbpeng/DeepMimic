#pragma once

#include "anim/MotionController.h"

class cClipsController : public cMotionController
{
public:

	cClipsController();
	virtual ~cClipsController();

	virtual void Init(cKinCharacter* character, const std::string& param_file);
	virtual void Reset();

	virtual int GetCurrMotionID() const;
	virtual const cMotion& GetMotion() const;
	virtual cMotion& GetMotion();
	virtual const cMotion& GetMotion(int m) const;
	virtual cMotion& GetMotion(int m);
	virtual int GetNumMotions() const;
	virtual void ChangeMotionDuration(double dur);
	virtual double GetMotionWeight(int m) const;

	virtual void EnableRandMotions(bool enable);
	virtual void ChangeMotion(int motion_id);
	virtual int SampleMotionID() const;
	virtual int SampleMotionID(const Eigen::VectorXd& clips_cdf) const;

protected:
	struct tMotionEntry
	{
		cMotion mMotion;
		double mWeight;
		tMotionEntry();
	};

	std::vector<tMotionEntry> mMotions;
	Eigen::VectorXd mClipsCDF;
	int mCurrMotionID;
	bool mEnableRandMotions;

	virtual bool LoadParams(const std::string& param_file);
	virtual bool LoadMotions(const Json::Value& json);
	virtual double CalcTotalMotionDur() const;
	virtual void BuildClipsCDF();

	virtual void ResetParams();

	virtual int SelectNewMotion() const;
	virtual int SelectNewMotion(const Eigen::VectorXd& clips_cdf) const;
	virtual void ActivateMotion(int motion_id);
};