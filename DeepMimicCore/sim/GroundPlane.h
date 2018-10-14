#pragma once

#include "Ground.h"

class cGroundPlane : public cGround
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static bool ParseParamsJson(const Json::Value& json, Eigen::VectorXd& out_params);

	cGroundPlane();
	virtual ~cGroundPlane();

	virtual void Init(const std::shared_ptr<cWorld>& world, const tParams& params);
	virtual void Update(double time_elapsed, const tVector& bound_min, const tVector& bound_max);
	virtual void Clear();

	virtual double SampleHeight(const tVector& pos) const;
	virtual double SampleHeight(const tVector& pos, bool& out_valid_sample) const;
	virtual void SampleHeight(const Eigen::MatrixXd& pos, Eigen::VectorXd& out_h) const;

	virtual const tVector& GetPrevCenter() const;
	virtual eClass GetGroundClass() const;

	virtual void CalcAABB(tVector& out_min, tVector& out_max) const;

protected:
	tVector mPrevCenter;
};