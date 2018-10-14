#pragma once

#include <vector>
#include "util/MathUtil.h"

class cMatrixStack
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cMatrixStack(size_t capacity);
	virtual ~cMatrixStack();

	virtual void Clear();
	virtual const tMatrix& GetMatrix() const;
	virtual void Push();
	virtual void Push(const tMatrix& mat);
	virtual void Pop();

	virtual size_t GetCapacity() const;
	virtual size_t GetSize() const;

	virtual void SetIdentity();
	virtual void SetMatrix(const tMatrix& mat);
	virtual void MultMatrix(const tMatrix& mat); // right multiply
	virtual void Translate(const tVector& trans);
	virtual void Scale(const tVector& scale);
	virtual void Rotate(double theta, const tVector& axis);

protected:

	int mStackHead;
	tMatrix mCurrMat;
	std::vector<tMatrix, Eigen::aligned_allocator<tMatrix>> mMatStack;
};