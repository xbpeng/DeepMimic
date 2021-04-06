#include "Motion.h"
#include <assert.h>
#include <iostream>

#include "util/FileUtil.h"
#include "util/JsonUtil.h"

const double gMinTime = 0;

// Json keys
const std::string gFrameKey = "Frames";
const std::string gLoopKey = "Loop";
const std::string gVelFilterCutoffKey = "VelFilterCutoff";
const std::string gCycleSyncRootPosKey = "CycleSyncRootPos";
const std::string gCycleSyncRootRotKey = "CycleSyncRootRot";
const std::string gCycleSyncRootHeightKey = "CycleSyncRootHeight";
const std::string gRightJointsKey = "RightJoints";
const std::string gLeftJointsKey = "LeftJoints";

const std::string gLoopStr[cMotion::eLoopMax] =
{
	"none",
	"wrap"
};

std::string cMotion::BuildFrameJson(const Eigen::VectorXd& frame)
{
	std::string json = cJsonUtil::BuildVectorJson(frame);
	return json;
}

double cMotion::CalcPhase(double time, double period, double phase_offset, bool loop)
{
	double phase = time / period;
	phase += phase_offset;

	if (loop)
	{
		phase -= std::floor(phase);
	}
	else
	{
		phase = cMathUtil::Clamp(phase, 0.0, 1.0);
	}

	return phase;
}

double cMotion::CalcFixPhaseTimeOffset(double time, double prev_period, double new_period,
										double time_offset, double phase_offset, bool loop)
{
	double prev_phase = CalcPhase(time + time_offset, prev_period, phase_offset, loop);
	double new_phase = CalcPhase(time + time_offset, new_period, phase_offset, loop);

	double dphase = prev_phase - new_phase;
	double dt = dphase * new_period;

	double new_time_offset = time_offset + dt;
	if (new_time_offset < 0)
	{
		new_time_offset += new_period;
	}
	assert(new_time_offset >= 0);

	return new_time_offset;
}

cMotion::tParams::tParams()
{
	mMotionFile = "";
	mBlendFunc = nullptr;
	mVelFunc = nullptr;
	mPostProcessFunc = nullptr;
};

cMotion::cMotion()
{
	Clear();
	mLoop = eLoopNone;
	mCycleSyncRootPos = true;
	mCycleSyncRootRot = false;
	mCycleSyncRootHeight = false;

	mCycleDeltaRootPos.setZero();
	mCycleDeltaRootHeading = 0.0;

	mFrameTimes.resize(0);
	mFrames.resize(0, 0);
}

cMotion::~cMotion()
{

}

void cMotion::Clear()
{
	mLoop = eLoopNone;
	mFrames.resize(0, 0);
	mFrameVel.resize(0, 0);
	mParams = tParams();
}

bool cMotion::Load(const tParams& params)
{
	Clear();
	
	mParams = params;
	std::ifstream f_stream(mParams.mMotionFile);
	Json::Value root;
	Json::Reader reader;
	bool succ = reader.parse(f_stream, root);
	f_stream.close();

	if (succ)
	{
		succ = LoadJson(root);
		if (succ)
		{
			PostProcessFrames(mFrameTimes, mFrames);
			UpdateVel();
			mCycleDeltaRootPos = CalcCycleDeltaRootPos();
			mCycleDeltaRootHeading = CalcCycleDeltaRootHeading();

			double motion_dur = GetDuration();
			printf("Loaded %.3f seconds of motion data from %s.\n", motion_dur, mParams.mMotionFile.c_str());
		}
		else
		{
			printf("Failed to load motion from file %s\n", mParams.mMotionFile.c_str());
			assert(false);
		}
	}
	else
	{
		printf("Failed to parse Json from %s\n", mParams.mMotionFile.c_str());
		assert(false);
	}

	return succ;
}

void cMotion::Init(int num_frames, int num_dofs)
{
	Clear();
	mFrames.resize(num_frames, num_dofs);
	mFrameTimes.resize(num_frames);
}

bool cMotion::IsValid() const
{
	return GetNumFrames() > 0;
}

int cMotion::GetNumDof() const
{
	return GetFrameSize();
}

int cMotion::GetNumFrames() const
{
	return static_cast<int>(mFrames.rows());
}

int cMotion::GetFrameSize() const
{
	return static_cast<int>(mFrames.cols());
}

void cMotion::BuildFrameVel(Eigen::MatrixXd& out_frame_vel) const
{
	int num_frames = GetNumFrames();
	int dof = GetNumDof();
	out_frame_vel = Eigen::MatrixXd::Zero(num_frames, dof);

	Eigen::VectorXd vel;
	for (int f = 0; f < num_frames - 1; ++f)
	{
		double dt = GetFrameDuration(f);
		Eigen::VectorXd frame0 = GetFrame(f);
		Eigen::VectorXd frame1 = GetFrame(f + 1);

		CalcFrameVel(frame0, frame1, dt, vel);
		out_frame_vel.row(f) = vel;
	}

	if (num_frames > 1)
	{
		out_frame_vel.row(num_frames - 1) = out_frame_vel.row(num_frames - 2);
	}
}

tVector cMotion::CalcCycleDeltaRootPos() const
{
	int num_frames = GetNumFrames();
	Eigen::VectorXd frame_beg = GetFrame(0);
	Eigen::VectorXd frame_end = GetFrame(num_frames - 1);

	tVector root_pos_beg = cKinTree::GetRootPos(frame_beg);
	tVector root_pos_end = cKinTree::GetRootPos(frame_end);

	tVector delta_pos = root_pos_end - root_pos_beg;

	if (!mCycleSyncRootHeight)
	{
		delta_pos[1] = 0.0;
	}

	return delta_pos;
}

double cMotion::CalcCycleDeltaRootHeading() const
{
	int num_frames = GetNumFrames();
	Eigen::VectorXd frame_beg = GetFrame(0);
	Eigen::VectorXd frame_end = GetFrame(num_frames - 1);

	tQuaternion root_rot_beg = cKinTree::GetRootRot(frame_beg);
	tQuaternion root_rot_end = cKinTree::GetRootRot(frame_end);

	tQuaternion delta_rot = cMathUtil::QuatDiff(root_rot_beg, root_rot_end);
	double delta_heading = cKinTree::CalcHeading(delta_rot);

	return delta_heading;
}

cMotion::tFrame cMotion::GetFrame(int f) const
{
	return mFrames.row(f);
}

cMotion::tFrame cMotion::GetFrameVel(int f) const
{
	return mFrameVel.row(f);
}

void cMotion::SetFrame(int f, const tFrame& frame)
{
	int frame_size = GetFrameSize();
	assert(frame.size() == frame_size);
	mFrames.row(f) = frame;
}

void cMotion::SetFrameTime(int f, double time)
{
	mFrameTimes(f) = time;
}

void cMotion::BlendFrames(int a, int b, double lerp, tFrame& out_frame) const
{
	lerp = cMathUtil::Saturate(lerp);

	// remove time params
	tFrame frame0 = GetFrame(a);
	tFrame frame1 = GetFrame(b);

	if (HasBlendFunc())
	{
		mParams.mBlendFunc(&frame0, &frame1, lerp, &out_frame);
	}
	else
	{
		BlendFramesIntern(&frame0, &frame1, lerp, &out_frame);
	}
}

void cMotion::CalcFrame(double time, tFrame& out_frame) const
{
	int idx;
	double blend;
	CalcIndexBlend(time, idx, blend);

	BlendFrames(idx, idx + 1, blend, out_frame);
}

void cMotion::CalcFrameVel(double time, cMotion::tFrame& out_vel) const
{
	if (!EnableLoop() && time >= GetDuration())
	{
		out_vel = Eigen::VectorXd::Zero(GetNumDof());
	}
	else
	{
		const Eigen::MatrixXd* vel_frame = &mFrameVel;

		int idx;
		double blend;
		CalcIndexBlend(time, idx, blend);
		auto vel0 = vel_frame->row(idx);
		auto vel1 = vel_frame->row(idx + 1);
		cKinTree::LerpVels(vel0, vel1, blend, out_vel);
	}
}

void cMotion::CalcFramePhase(double phase, tFrame& out_frame) const
{
	double max_time = GetDuration();
	double time = phase * max_time;
	CalcFrame(time, out_frame);
}

bool cMotion::LoadJson(const Json::Value& root)
{
	bool succ = true;
	if (!root[gLoopKey].isNull())
	{
		std::string loop_str = root[gLoopKey].asString();
		ParseLoop(loop_str, mLoop);
	}

	mCycleSyncRootPos = root.get(gCycleSyncRootPosKey, mCycleSyncRootPos).asBool();
	mCycleSyncRootRot = root.get(gCycleSyncRootRotKey, mCycleSyncRootRot).asBool();
	mCycleSyncRootHeight = root.get(gCycleSyncRootHeightKey, mCycleSyncRootHeight).asBool();

	if (!root[gFrameKey].isNull())
	{
		succ &= LoadJsonFrames(root[gFrameKey], mFrameTimes, mFrames);
	}
	return succ;
}

bool cMotion::ParseLoop(const std::string& str, eLoop& out_loop) const
{
	bool succ = false;
	for (int i = 0; i < eLoopMax; ++i)
	{
		if (str == gLoopStr[i])
		{
			out_loop = static_cast<eLoop>(i);
			succ = true;
			break;
		}
	}
	
	if (!succ)
	{
		printf("Unsupported loop mode: %s\n", str.c_str());
		assert(false);
		succ = false;
	}
	return succ;
}

bool cMotion::LoadJsonFrames(const Json::Value& root, Eigen::VectorXd& out_frame_times, Eigen::MatrixXd& out_frames) const
{
	bool succ = true;

	assert(root.isArray());
	int num_frames = root.size();

	int frame_size = 0;
	if (num_frames > 0)
	{
		int idx0 = 0;
		Json::Value frame_json = root.get(idx0, 0);
		frame_size = frame_json.size() - 1; // first entry is the duration
		out_frame_times.resize(num_frames);
		out_frames.resize(num_frames, frame_size);
	}

	for (int f = 0; f < num_frames; ++f)
	{
		Eigen::VectorXd curr_frame_data;
		succ &= ParseFrameJson(root.get(f, 0), curr_frame_data);
		if (succ)
		{
			assert(out_frames.cols() == curr_frame_data.size() - 1);
			out_frame_times(f) = curr_frame_data[0];
			out_frames.row(f) = curr_frame_data.segment(1, frame_size);
		}
		else
		{
			out_frames.resize(0, 0);
			break;
		}
	}
	return succ;
}

bool cMotion::ParseFrameJson(const Json::Value& root, Eigen::VectorXd& out_frame) const
{
	bool succ = false;
	if (root.isArray())
	{
		int data_size = root.size();
		out_frame.resize(data_size);
		for (int i = 0; i < data_size; ++i)
		{
			Json::Value json_elem = root.get(i, 0);
			out_frame[i] = json_elem.asDouble();
		}

		succ = true;
	}
	return succ;
}

std::string cMotion::BuildLoopStr(eLoop loop) const
{
	return gLoopStr[loop];
}

void cMotion::PostProcessFrames(Eigen::VectorXd& frame_times, Eigen::MatrixXd& frames) const
{
	int frame_size = GetFrameSize();
	int num_frames = static_cast<int>(frames.rows());
	double curr_time = gMinTime;

	Eigen::VectorXd root_pos_offset = cKinTree::GetRootPos(frames.row(0));
	root_pos_offset[1] = 0;

	for (int f = 0; f < num_frames; ++f)
	{
		Eigen::VectorXd curr_frame = frames.row(f);
		double duration = frame_times(f);
		frame_times(f) = curr_time;
		curr_time += duration;

		if (HasPostProcessFunc())
		{
			// center start of motion at origin
			tVector curr_root_pos = cKinTree::GetRootPos(curr_frame);
			curr_root_pos -= root_pos_offset;
			cKinTree::SetRootPos(curr_root_pos, curr_frame);
			mParams.mPostProcessFunc(&curr_frame);

			frames.row(f) = curr_frame;
		}
	}
}

double cMotion::GetDuration() const
{
	int num_frames = GetNumFrames();
	double max_time = mFrameTimes(num_frames - 1);
	return max_time;
}

void cMotion::ChangeDuration(double dur)
{
	double dur_old = GetDuration();
	int num_frames = GetNumFrames();

	for (int f = 0; f < num_frames; ++f)
	{
		double t = GetFrameTime(f);
		t /= dur_old;
		t *= dur;
		SetFrameTime(f, t);
	}
	UpdateVel();
}

double cMotion::GetFrameTime(int f) const
{
	return mFrameTimes(f);
}

double cMotion::GetFrameDuration(int f) const
{
	double dur = 0;
	if (f < GetNumFrames() - 1)
	{
		dur = GetFrameTime(f + 1) - GetFrameTime(f);
	}
	return dur;
}

double cMotion::CalcPhase(double time) const
{
	double dur = GetDuration();
	double phase = CalcPhase(time, dur, 0, EnableLoop());
	return phase;
}

int cMotion::CalcCycleCount(double time) const
{
	double dur = GetDuration();
	double phase = time / dur;
	int count = static_cast<int>(std::floor(phase));
	bool loop = EnableLoop();
	count = (loop) ? count : cMathUtil::Clamp(count, 0, 1);
	return count;
}

void cMotion::CalcIndexBlend(double time, int& out_idx, double& out_blend) const
{
	double max_time = GetDuration();

	if (!EnableLoop())
	{
		if (time <= gMinTime)
		{
			out_idx = 0;
			out_blend = 0;
			return;
		}
		else if (time >= max_time)
		{
			out_idx = GetNumFrames() - 2;
			out_blend = 1;
			return;
		}
	}

	int cycle_count = CalcCycleCount(time);
	time -= cycle_count * GetDuration();

	auto it = std::upper_bound(mFrameTimes.data(), mFrameTimes.data() + mFrameTimes.size(), time);
	out_idx = static_cast<int>(it - mFrameTimes.data() - 1);

	double time0 = mFrameTimes(out_idx);
	double time1 = mFrameTimes(out_idx + 1);
	out_blend = (time - time0) / (time1 - time0);
}

void cMotion::UpdateVel()
{
	BuildFrameVel(mFrameVel);
}

void cMotion::TimeWrap(double scale)
{
	// linear timewarp
	double duration = GetDuration();
	ChangeDuration(scale * duration);
}

bool cMotion::IsOver(double time) const
{
	return !EnableLoop() && (time >= GetDuration());
}

bool cMotion::HasBlendFunc() const
{
	return mParams.mBlendFunc != nullptr;
}

bool cMotion::HasVelFunc() const
{
	return mParams.mVelFunc != nullptr;
}

bool cMotion::HasPostProcessFunc() const
{
	return mParams.mPostProcessFunc != nullptr;
}

bool cMotion::EnableLoop() const
{
	return mLoop != eLoopNone;
}

cMotion::eLoop cMotion::GetLoop() const
{
	return mLoop;
}

void cMotion::SetLoop(eLoop loop)
{
	mLoop = loop;
}

void cMotion::BlendFramesIntern(const Eigen::VectorXd* a, const Eigen::VectorXd* b, double lerp, cMotion::tFrame* out_frame) const
{
	*out_frame = (1 - lerp) * (*a) + lerp * (*b);
}

void cMotion::CalcFrameVel(const tFrame& frame0, const tFrame& frame1, double dt, tFrame& out_vel) const
{
	if (HasVelFunc())
	{
		mParams.mVelFunc(&frame0, &frame1, dt, &out_vel);
	}
	else
	{
		out_vel = (frame1 - frame0) / dt;
	}
}

void cMotion::Output(const std::string& out_filepath) const
{
	FILE* file = cFileUtil::OpenFile(out_filepath, "w");

	fprintf(file, "{\n");
	fprintf(file, "\"%s\": ", gLoopKey.c_str());

	std::string loop_str = BuildLoopStr(mLoop);
	fprintf(file, "\"%s\",\n", loop_str.c_str());

	if (mCycleSyncRootPos)
	{
		fprintf(file, "\"%s\": true,\n", gCycleSyncRootPosKey.c_str());
	}
	else
	{
		fprintf(file, "\"%s\": false,\n", gCycleSyncRootPosKey.c_str());
	}

	if (mCycleSyncRootRot)
	{
		fprintf(file, "\"%s\": true,\n", gCycleSyncRootRotKey.c_str());
	}
	else
	{
		fprintf(file, "\"%s\": false,\n", gCycleSyncRootRotKey.c_str());
	}

	if (mCycleSyncRootHeight)
	{
		fprintf(file, "\"%s\": true,\n", gCycleSyncRootHeightKey.c_str());
	}
	else
	{
		fprintf(file, "\"%s\": false,\n", gCycleSyncRootHeightKey.c_str());
	}

	fprintf(file, "\n");
	fprintf(file, "\"Frames\":\n[\n");

	int num_frames = GetNumFrames();
	int frame_size = GetFrameSize();
	Eigen::VectorXd frame_data = Eigen::VectorXd::Zero(frame_size + 1);
	for (int f = 0; f < num_frames; ++f)
	{
		if (f != 0)
		{
			fprintf(file, ",\n");
		}

		Eigen::VectorXd curr_frame = mFrames.row(f);
		double dur = 0;
		if (f < num_frames - 1)
		{
			dur = GetFrameDuration(f);
		}
		frame_data[0] = dur;
		frame_data.segment(1, frame_size) = curr_frame;
		std::string frame_json = cJsonUtil::BuildVectorJson(frame_data);
		fprintf(file, "%s", frame_json.c_str());
	}

	fprintf(file, "\n]");
	fprintf(file, "\n}");
	cFileUtil::CloseFile(file);
}