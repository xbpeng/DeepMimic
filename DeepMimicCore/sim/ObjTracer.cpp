#include "ObjTracer.h"
#include "render/DrawUtil.h"

const int gNumEndEffMarkers = 4;
const double gMarkerSize = 0.075;

cObjTracer::tParams::tParams()
{
	mType = eTracePos;
	mObj = nullptr;
	mColors.resize(1);
	mColors[0] = tVector(0, 0, 1, 0.5);
	mLocalPos.setZero();
	mShapeIdx = 0;
	mBufferSize = 300;
}

bool cObjTracer::tParams::IsValid() const
{
	bool valid = mObj != nullptr;
	return valid;
}

cObjTracer::cObjTracer()
{
}

cObjTracer::~cObjTracer()
{

}

void cObjTracer::Init(double sample_period)
{
	Clear();

	cTimer::tParams timer_params;
	timer_params.mType = cTimer::eTypeUniform;
	timer_params.mTimeMin = sample_period;
	timer_params.mTimeMax = sample_period;
	mTimer.Init(timer_params);
}

void cObjTracer::Update(double time_step)
{
	mTimer.Update(time_step);

	for (int i = 0; i < GetNumTraces(); ++i)
	{
		UpdateTrace(mTraces[i]);
	}

	if (mTimer.IsEnd())
	{
		ResetTimer();
	}
}

void cObjTracer::Reset()
{
	ResetTimer();
	for (int i = 0; i < GetNumTraces(); ++i)
	{
		ResetTrace(mTraces[i]);
	}
}

void cObjTracer::Clear()
{
	ResetTimer();
	mTraces.clear();
}

int cObjTracer::AddTrace(const tParams& params)
{
	int handle = gInvalidIdx;
	if (params.IsValid())
	{
		tTrace trace;
		BuildTrace(params, trace);

		handle = GetNumTraces();
		mTraces.push_back(trace);
	}
	else
	{
		assert(false); // invalid trace parameters
	}
	return handle;
}

void cObjTracer::SetTraceColIdx(int handle, int col_idx)
{
	tTrace& trace = GetTrace(handle);
	SetTraceColIdx(col_idx, trace);
}

int cObjTracer::GetNumTraces() const
{
	return static_cast<int>(mTraces.size());
}

void cObjTracer::Draw() const
{
	for (int i = 0; i < GetNumTraces(); ++i)
	{
		DrawTrace(mTraces[i]);
	}
}

void cObjTracer::ResetTimer()
{
	mTimer.Reset();
}

void cObjTracer::BuildTrace(const tParams& params, tTrace& out_trace) const
{
	out_trace.mParams = params;
	out_trace.mInContact = false;
	out_trace.mSampleBuffer.Reserve(params.mBufferSize);
}

void cObjTracer::ResetTrace(tTrace& out_trace) const
{
	out_trace.mInContact = false;
	SetTraceColIdx(0, out_trace);
	out_trace.mSampleBuffer.Clear();
}

void cObjTracer::UpdateTrace(tTrace& out_trace) const
{
	switch (out_trace.mParams.mType)
	{
	case eTracePos:
		UpdateTracePos(out_trace);
		break;
	case eTraceCOM:
		UpdateTraceCOM(out_trace);
		break;
	case eTraceContact:
		UpdateTraceContact(out_trace);
		break;
	default:
		printf("Unsupported trace type\n");
		assert(false);
	}
	out_trace.mInContact = out_trace.mParams.mObj->IsInContact();
}

void cObjTracer::UpdateTracePos(tTrace& out_trace) const
{
	if (mTimer.IsEnd())
	{
		tVector pos = out_trace.mParams.mObj->LocalToWorldPos(out_trace.mParams.mLocalPos);
		pos[3] = out_trace.mColIdx;
		out_trace.mSampleBuffer.Add(pos);
	}
}

void cObjTracer::UpdateTraceCOM(tTrace& out_trace) const
{
	if (mTimer.IsEnd())
	{
		tVector pos = out_trace.mParams.mObj->CalcCOM();
		pos[3] = out_trace.mColIdx;
		out_trace.mSampleBuffer.Add(pos);
	}
}

void cObjTracer::UpdateTraceContact(tTrace& out_trace) const
{
	const auto& obj = out_trace.mParams.mObj;
	bool contact = obj->IsInContact();
	bool prev_contact = out_trace.mInContact;
	if (contact && !prev_contact)
	{
		const auto& pts = obj->GetContactPts();
		tVector pos = pts[0].mPos;
		pos[3] = out_trace.mColIdx;
		out_trace.mSampleBuffer.Add(pos);
	}
}

const cObjTracer::tTrace& cObjTracer::GetTrace(int handle) const
{
	assert(handle >= 0 && handle < GetNumTraces());
	return mTraces[handle];
}

cObjTracer::tTrace& cObjTracer::GetTrace(int handle)
{
	assert(handle >= 0 && handle < GetNumTraces());
	return mTraces[handle];
}

void cObjTracer::DrawTrace(const tTrace& trace) const
{
	cDrawUtil::SetLineWidth(3);
	switch (trace.mParams.mType)
	{
	case eTracePos:
	case eTraceCOM:
		DrawTracePos(trace);
		break;
	case eTraceContact:
		DrawTraceContact(trace);
		break;
	default:
		printf("Unsupported trace type\n");
		assert(false);
	}
}

void cObjTracer::DrawTracePos(const tTrace& trace) const
{
	if (trace.mSampleBuffer.GetSize() > 1)
	{
		cDrawUtil::LoadShaderUniforms();
		glBegin(GL_LINE_STRIP);
		for (size_t i = 0; i < trace.mSampleBuffer.GetSize(); ++i)
		{
			const tVector& vert0 = trace.mSampleBuffer[i];
			int curr_col_idx = static_cast<int>(vert0[3]);

			assert(curr_col_idx < trace.mParams.mColors.size());
			const tVector& col = trace.mParams.mColors[curr_col_idx];
			cDrawUtil::SetColor(col);
			glVertex3d(vert0[0], vert0[1], vert0[2]);
		}
		glEnd();
	}
}

void cObjTracer::DrawTraceContact(const tTrace& trace) const
{
	const double marker_size = gMarkerSize;

	int shape_idx = trace.mParams.mShapeIdx;
	shape_idx = shape_idx % gNumEndEffMarkers;

	for (int i = 0; i < static_cast<int>(trace.mSampleBuffer.GetSize()); ++i)
	{
		const auto& pos = trace.mSampleBuffer[i];
		int curr_col_idx = static_cast<int>(pos[3]);
		assert(curr_col_idx < trace.mParams.mColors.size());
		const tVector& col = trace.mParams.mColors[curr_col_idx];
		cDrawUtil::SetColor(col);

		switch (shape_idx)
		{
		case 0:
			cDrawUtil::DrawRect(pos, tVector(marker_size, marker_size, 0, 0));
			break;
		case 1:
			cDrawUtil::DrawTriangle(pos, 1.25 * marker_size);
			break;
		case 2:
			cDrawUtil::DrawCross(pos, marker_size);
			break;
		case 3:
			cDrawUtil::DrawDisk(pos, 0.5 * marker_size);
			break;
		default:
			assert(false);
			break;
		}
	}
}

void cObjTracer::SetTraceColIdx(int idx, tTrace& out_trace) const
{
	int num_cols = static_cast<int>(out_trace.mParams.mColors.size());
	idx = idx % num_cols;
	out_trace.mColIdx = idx;
}
