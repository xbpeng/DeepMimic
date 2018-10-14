#pragma once
#include <memory>
#include "util/CircularBuffer.h"
#include "sim/SimObj.h"
#include "util/Timer.h"

class cObjTracer
{
public:
	enum eTraceType
	{
		eTracePos,
		eTraceCOM,
		eTraceContact,
		eTrceMax
	};

	struct tParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		eTraceType mType;
		std::shared_ptr<cSimObj> mObj;
		tVectorArr mColors;
		tVector mLocalPos;
		int mShapeIdx;
		int mBufferSize;

		tParams();
		bool IsValid() const;
	};

	cObjTracer();
	virtual ~cObjTracer();

	virtual void Init(double sample_period);
	virtual void Update(double time_step);
	virtual void Reset();
	virtual void Clear();

	virtual int AddTrace(const tParams& params);
	virtual int GetNumTraces() const;
	virtual void SetTraceColIdx(int handle, int col_idx);

	virtual void Draw() const;

protected:
	struct tTrace
	{
		tParams mParams;
		bool mInContact;
		int mColIdx;
		cCircularBuffer<tVector, Eigen::aligned_allocator<tVector>> mSampleBuffer;
	};

	cTimer mTimer;
	tEigenArr<tTrace> mTraces;

	virtual void ResetTimer();
	virtual void BuildTrace(const tParams& params, tTrace& out_trace) const;
	virtual void ResetTrace(tTrace& out_trace) const;
	virtual void UpdateTrace(tTrace& out_trace) const;
	virtual void UpdateTracePos(tTrace& out_trace) const;
	virtual void UpdateTraceCOM(tTrace& out_trace) const;
	virtual void UpdateTraceContact(tTrace& out_trace) const;

	virtual const tTrace& GetTrace(int handle) const;
	virtual tTrace& GetTrace(int handle);

	virtual void DrawTrace(const tTrace& trace) const;
	virtual void DrawTracePos(const tTrace& trace) const;
	virtual void DrawTraceContact(const tTrace& trace) const;

	virtual void SetTraceColIdx(int idx, tTrace& out_trace) const;
};