#pragma once

#include "Controller.h"
#include "anim/Motion.h"
#include "sim/ActionSpace.h"

class cCharController : public cController
{
public:
	// norm groups are used to determine how different inputs should be normalized during training
	// gNormGroupSingle means that the feature is normalized independently of other features
	// gNormGroupNone means that no normalization is performed on the feature
	// any other value will group the features of the same group so that all features in a group 
	// are normalized by the same offset and scale
	// these group IDs must be the same as those in normalizer.py
	const static int gNormGroupSingle;
	const static int gNormGroupNone;

	virtual ~cCharController();

	virtual void Update(double time_step);
	virtual void PostUpdate(double time_step);
	
	virtual bool NeedNewAction() const;
	virtual void ApplyAction(const Eigen::VectorXd& action);
	virtual void RecordState(Eigen::VectorXd& out_state);
	virtual void RecordAction(Eigen::VectorXd& out_action) const;

	virtual eActionSpace GetActionSpace() const = 0;
	virtual int GetStateSize() const;
	virtual int GetActionSize() const;
	virtual int GetNumActions() const;

	virtual void HandlePoseReset();
	virtual void HandleVelReset();

	virtual void BuildStateOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildActionOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;
	
	virtual void BuildStateNormGroups(Eigen::VectorXi& out_groups) const;
	
	virtual double GetRewardMin() const;
	virtual double GetRewardMax() const;

	virtual int NumChildren() const;
	virtual const std::shared_ptr<cCharController>& GetChild(int i) const;

	virtual std::string GetName() const = 0;

protected:

	cCharController();
};
