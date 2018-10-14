#pragma once

#include "util/MathUtil.h"

class cTrajectory
{
public:
	enum eWrapMode
	{
		eWrapModeClamp,
		eWrapModeLoop,
		eWrapModeMax,
	};

	cTrajectory();
	virtual ~cTrajectory();

	virtual void Init(const Eigen::MatrixXd& anchors, eWrapMode mode);
	virtual void Clear();
	virtual Eigen::VectorXd Eval(double t);
	virtual int GetNumAnchors() const;
	virtual int GetDim() const;
	virtual int GetSize() const;
	virtual Eigen::VectorXd GetAnchor(int idx) const;
	virtual void SetAnchor(const Eigen::VectorXd& anchor, int idx);
	virtual double PhaseToAnchor(double t) const;
	virtual double AnchorToPhase(int idx) const;

	// standard deviation (wrt phase) of gaussian kernel used to filter trajectory
	virtual void Filter(double r);
	virtual void Compress(double ratio);
	virtual Eigen::VectorXd Unroll() const;
	virtual void SetUnrolled(const Eigen::VectorXd& unrolled_data, int dim);

protected:
	eWrapMode mWrapMode;
	Eigen::MatrixXd mAnchors;

	virtual double NormalizePhase(double t) const;
	virtual double GetPhaseStep() const;
};