#pragma once

#include "util/MathUtil.h"

class cDynamicTimeWarper
{
public:
	typedef std::function<double(const Eigen::VectorXd* data0, const Eigen::VectorXd* data1)> tCostFunc;

	static double DefaultCost(const Eigen::VectorXd* data0, const Eigen::VectorXd* data1);

	cDynamicTimeWarper();
	virtual ~cDynamicTimeWarper();

	virtual void Init(int dim0, int dim1, int buffer_size, tCostFunc cost_func=nullptr);
	virtual void Reset();
	virtual int GetBufferSize();
	virtual int GetDim0() const;
	virtual int GetDim1() const;
	virtual int GetSampleCount0() const;
	virtual int GetSampleCount1() const;

	virtual void AddSample0(const Eigen::VectorXd& data);
	virtual void AddSample1(const Eigen::VectorXd& data);
	virtual Eigen::VectorXd GetSample0(int i) const;
	virtual Eigen::VectorXd GetSample1(int i) const;

	virtual double CalcAlignment();
	virtual const Eigen::VectorXi& CalcMinCostPath();
	virtual double GetMinAlignmentCost() const;
	virtual const Eigen::VectorXi& GetMinCostPath() const;

protected:
	tCostFunc mCostFunc;

	int mSampleCount0;
	int mSampleCount1;
	Eigen::MatrixXd mData0;
	Eigen::MatrixXd mData1;
	Eigen::MatrixXd mCostBuffer;
	Eigen::VectorXi mIndexBuffer;

	virtual double CalcCost(const Eigen::VectorXd& data0, const Eigen::VectorXd& data1) const;
};