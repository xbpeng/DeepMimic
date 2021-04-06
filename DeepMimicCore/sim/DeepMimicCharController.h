#pragma once

#include "sim/CharController.h"
#include "sim/Ground.h"
#include "util/CircularBuffer.h"

class cDeepMimicCharController : public cCharController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	virtual ~cDeepMimicCharController();

	virtual void Init(cSimCharacter* character, const std::string& param_file);
	virtual void Reset();
	virtual void Clear();

	virtual void Update(double timestep);
	virtual void PostUpdate(double timestep);
	virtual void UpdateCalcTau(double timestep, Eigen::VectorXd& out_tau);
	virtual void UpdateApplyTau(const Eigen::VectorXd& tau);

	virtual void SetGround(std::shared_ptr<cGround> ground);

	virtual bool NeedNewAction() const;
	virtual void ApplyAction(const Eigen::VectorXd& action);
	virtual void RecordState(Eigen::VectorXd& out_state);
	virtual void RecordAction(Eigen::VectorXd& out_action) const;

	virtual eActionSpace GetActionSpace() const;
	virtual int GetStateSize() const;

	virtual double GetRewardMin() const;
	virtual double GetRewardMax() const;
	
	virtual void SetViewDistMin(double dist);
	virtual void SetViewDistMax(double dist);
	virtual double GetViewDistMin() const;
	virtual double GetViewDistMax() const;
	
	virtual void GetViewBound(tVector& out_min, tVector& out_max) const;

	virtual double GetPrevActionTime() const;
	virtual const tVector& GetPrevActionCOM() const;
	virtual double GetTime() const;

	virtual const Eigen::VectorXd& GetTau() const;
	virtual const cCircularBuffer<double>& GetValLog() const;
	virtual void LogVal(double val);

protected:
	double mTime;
	bool mNeedNewAction;
	Eigen::VectorXd mAction;
	Eigen::VectorXd mTau;

	double mViewDistMin;
	double mViewDistMax;
	int mPosDim;

	double mPrevActionTime;
	tVector mPrevActionCOM;

	std::shared_ptr<cGround> mGround;

	// for recording prediction from the value function, mainly for visualization
	cCircularBuffer<double> mValLog; 

	cDeepMimicCharController();

	virtual bool ParseParams(const Json::Value& json);

	virtual void ResetParams();
	virtual void InitResources();
	virtual void InitAction();
	virtual void InitTau();
	
	virtual int GetPosDim() const;

	virtual bool CheckNeedNewAction(double timestep) const;
	virtual void NewActionUpdate();
	virtual void HandleNewAction();
	virtual void PostProcessAction(Eigen::VectorXd& out_action) const;
	virtual bool HasGround() const;
	virtual double SampleGroundHeight(const tVector& pos) const;

	virtual void BuildStatePose(Eigen::VectorXd& out_pose) const;
	virtual void BuildStateVel(Eigen::VectorXd& out_vel) const;
	virtual int GetStatePoseOffset() const;
	virtual int GetStateVelOffset() const;
	virtual int GetStatePoseSize() const;
	virtual int GetStateVelSize() const;
};