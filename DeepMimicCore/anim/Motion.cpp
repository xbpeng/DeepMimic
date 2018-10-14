#include "Motion.h"
#include <assert.h>
#include <iostream>

#include "util/FileUtil.h"
#include "util/JsonUtil.h"

const double gMinTime = 0;

// Json keys
const std::string cMotion::gFrameKey = "Frames";
const std::string cMotion::gLoopKey = "Loop";
const std::string cMotion::gVelFilterCutoffKey = "VelFilterCutoff";
const std::string gRightJointsKey = "RightJoints";
const std::string gLeftJointsKey = "LeftJoints";

const std::string gLoopStr[cMotion::eLoopMax] =
{
	"none",
	"wrap",
	"mirror"
};

std::string cMotion::BuildFrameJson(const Eigen::VectorXd& frame)
{
	std::string json = cJsonUtil::BuildVectorJson(frame);
	return json;
}

cMotion::tParams::tParams()
{
	mMotionFile = "";
	mBlendFunc = nullptr;
	mMirrorFunc = nullptr;
	mVelFunc = nullptr;
	mPostProcessFunc = nullptr;

	mRightJoints.clear();
	mLeftJoints.clear();
};

cMotion::cMotion()
{
	Clear();
	mLoop = eLoopNone;
	mVelFilterCutoff = 6; // cutoff 6 Hz
}

cMotion::~cMotion()
{

}

void cMotion::Clear()
{
	mFrames.resize(0, 0);
	mFrameVel.resize(0, 0);
	mFrameVelMirror.resize(0, 0);
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
			PostProcessFrames(mFrames);
			UpdateVel();
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
	mFrames = Eigen::MatrixXd::Zero(num_frames, num_dofs + eFrameMax);
}

bool cMotion::IsValid() const
{
	return GetNumFrames() > 0;
}

int cMotion::GetNumDof() const
{
	return GetFrameSize() - eFrameMax;
}

int cMotion::GetNumFrames() const
{
	return static_cast<int>(mFrames.rows());
}

int cMotion::GetFrameSize() const
{
	return static_cast<int>(mFrames.cols());
}

void cMotion::BuildFrameVel(Eigen::MatrixXd& out_frame_vel, bool mirror /*= false*/) const
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
		if (mirror)
		{
			MirrorFrame(frame0);
			MirrorFrame(frame1);
		}

		CalcFrameVel(frame0, frame1, dt, vel);
		out_frame_vel.row(f) = vel;
	}

	if (num_frames > 1)
	{
		out_frame_vel.row(num_frames - 1) = out_frame_vel.row(num_frames - 2);
	}

	if (mVelFilterCutoff > 0)
	{
		FilterFrameVel(out_frame_vel);
	}
}

void cMotion::FilterFrameVel(Eigen::MatrixXd& out_frame_vel) const
{
	double dt = GetFrameDuration(0);
	int num_dof = static_cast<int>(out_frame_vel.cols());

	for (int i = 0; i < num_dof; ++i)
	{
		Eigen::VectorXd x = out_frame_vel.col(i);
		cMathUtil::ButterworthFilter(dt, mVelFilterCutoff, x);
		out_frame_vel.col(i) = x;
	}
}

cMotion::tFrame cMotion::GetFrame(int i) const
{
	int frame_size = GetFrameSize();
	return mFrames.row(i).segment(eFrameMax, frame_size - eFrameMax);
}

void cMotion::SetFrame(int i, const tFrame& frame)
{
	int frame_size = GetFrameSize();
	assert(frame.size() == frame_size - eFrameMax);
	mFrames.row(i).segment(eFrameMax, frame_size - eFrameMax) = frame;
}

void cMotion::SetFrameTime(int i, double time)
{
	mFrames(i, 0) = time;
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

void cMotion::CalcFrame(double time, tFrame& out_frame, bool force_mirror /*=false*/) const
{
	int idx;
	double phase;
	CalcIndexPhase(time, idx, phase);

	BlendFrames(idx, idx + 1, phase, out_frame);
	if (NeedMirrorFrame(time) || force_mirror)
	{
		MirrorFrame(out_frame);
	}
}

void cMotion::CalcFrameVel(double time, cMotion::tFrame& out_vel, bool force_mirror /*=false*/) const
{
	if (!EnableLoop() && time >= GetDuration())
	{
		out_vel = Eigen::VectorXd::Zero(GetNumDof());
	}
	else
	{
		const Eigen::MatrixXd* vel_frame = nullptr;
		if (NeedMirrorFrame(time) || force_mirror)
		{
			vel_frame = &mFrameVelMirror;
		}
		else
		{
			vel_frame = &mFrameVel;
		}

		int idx;
		double phase;
		CalcIndexPhase(time, idx, phase);
		auto vel0 = vel_frame->row(idx);
		auto vel1 = vel_frame->row(idx + 1);
		out_vel = (1 - phase) * vel0 + phase * vel1;
	}
}

void cMotion::CalcFramePhase(double phase, tFrame& out_frame, bool force_mirror /*=false*/) const
{
	double max_time = GetDuration();
	double time = phase * max_time;
	CalcFrame(time, out_frame, force_mirror);
}

bool cMotion::LoadJson(const Json::Value& root)
{
	bool succ = true;
	if (!root[gLoopKey].isNull())
	{
		std::string loop_str = root[gLoopKey].asString();
		ParseLoop(loop_str, mLoop);
	}

	mVelFilterCutoff = root.get(gVelFilterCutoffKey, mVelFilterCutoff).asDouble();

	if (mParams.mRightJoints.size() == 0 && mParams.mLeftJoints.size() == 0)
	{
		succ &= LoadJsonJoints(root, mParams.mRightJoints, mParams.mLeftJoints);
	}
	assert(mParams.mRightJoints.size() == mParams.mLeftJoints.size());

	if (!root[gFrameKey].isNull())
	{
		succ &= LoadJsonFrames(root[gFrameKey], mFrames);
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

bool cMotion::LoadJsonFrames(const Json::Value& root, Eigen::MatrixXd& out_frames) const
{
	bool succ = true;

	assert(root.isArray());
	int num_frames = root.size();

	int data_size = 0;
	if (num_frames > 0)
	{
		int idx0 = 0;
		Json::Value frame_json = root.get(idx0, 0);
		data_size = frame_json.size();
		out_frames.resize(num_frames, data_size);
	}

	for (int f = 0; f < num_frames; ++f)
	{
		Eigen::VectorXd curr_frame;
		succ &= ParseFrameJson(root.get(f, 0), curr_frame);
		if (succ)
		{
			assert(mFrames.cols() == curr_frame.size());
			out_frames.row(f) = curr_frame;
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

bool cMotion::LoadJsonJoints(const Json::Value& root, std::vector<int>& out_right_joints, std::vector<int>& out_left_joints) const
{
	bool succ = true;
	if (!root[gRightJointsKey].isNull() && !root[gLeftJointsKey].isNull())
	{
		auto right_joints_json = root[gRightJointsKey];
		auto left_joints_json = root[gLeftJointsKey];
		assert(right_joints_json.isArray());
		assert(left_joints_json.isArray());

		int num_right_joints = right_joints_json.size();
		assert(num_right_joints == left_joints_json.size());
		succ = num_right_joints == left_joints_json.size();
		if (succ)
		{
			std::vector<int> right_joints(num_right_joints);
			std::vector<int> left_joints(num_right_joints);

			out_right_joints.resize(num_right_joints);
			out_left_joints.resize(num_right_joints);
			for (int i = 0; i < num_right_joints; ++i)
			{
				int right_id = right_joints_json[i].asInt();
				int left_id = left_joints_json[i].asInt();
				out_right_joints[i] = right_id;
				out_left_joints[i] = left_id;
			}
		}
	}
	else
	{
		out_right_joints.clear();
		out_left_joints.clear();
	}

	return succ;
}

std::string cMotion::BuildLoopStr(eLoop loop) const
{
	return gLoopStr[loop];
}

void cMotion::PostProcessFrames(Eigen::MatrixXd& frames) const
{
	int frame_size = GetFrameSize();
	int num_frames = static_cast<int>(frames.rows());
	double curr_time = gMinTime;

	for (int f = 0; f < num_frames; ++f)
	{
		auto curr_frame = frames.row(f);
		double duration = curr_frame(0, eFrameTime);
		curr_frame(0, eFrameTime) = curr_time;
		curr_time += duration;

		if (HasPostProcessFunc())
		{
			Eigen::VectorXd pose = curr_frame.segment(eFrameMax, frame_size - eFrameMax);
			mParams.mPostProcessFunc(&pose);
			curr_frame.segment(eFrameMax, frame_size - eFrameMax) = pose;
		}
	}
}

double cMotion::GetDuration() const
{
	int num_frames = GetNumFrames();
	double max_time = mFrames(num_frames - 1, eFrameTime);
	return max_time;
}

void cMotion::SetDuration(double dur)
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
	return mFrames(f, eFrameTime);
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

int cMotion::CalcCycleCount(double time) const
{
	double dur = GetDuration();
	double phases = time / dur;
	int count = static_cast<int>(std::floor(phases));
	bool loop = EnableLoop();
	count = (loop) ? count : cMathUtil::Clamp(count, 0, 1);
	return count;
}

void cMotion::CalcIndexPhase(double time, int& out_idx, double& out_phase) const
{
	double max_time = GetDuration();

	if (!EnableLoop())
	{
		if (time <= gMinTime)
		{
			out_idx = 0;
			out_phase = 0;
			return;
		}
		else if (time >= max_time)
		{
			out_idx = GetNumFrames() - 2;
			out_phase = 1;
			return;
		}
	}

	int cycle_count = CalcCycleCount(time);
	time -= cycle_count * GetDuration();
	if (time < 0)
	{
		time += max_time;
	}

	const Eigen::VectorXd& frame_times = mFrames.col(eFrameTime);
	auto it = std::upper_bound(frame_times.data(), frame_times.data() + frame_times.size(), time);
	out_idx = static_cast<int>(it - frame_times.data() - 1);

	double time0 = frame_times(out_idx);
	double time1 = frame_times(out_idx + 1);
	out_phase = (time - time0) / (time1 - time0);
}

void cMotion::UpdateVel()
{
	BuildFrameVel(mFrameVel, false);

	if (mLoop == eLoopMirror)
	{
		BuildFrameVel(mFrameVelMirror, true);
	}
}

bool cMotion::IsOver(double time) const
{
	return !EnableLoop() && (time >= GetDuration());
}

bool cMotion::HasBlendFunc() const
{
	return mParams.mBlendFunc != nullptr;
}

bool cMotion::HasMirrorFunc() const
{
	return mParams.mMirrorFunc != nullptr;
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

bool cMotion::NeedMirrorFrame(double time) const
{
	bool mirror = false;
	if (mLoop == eLoopMirror)
	{
		int cycle = CalcCycleCount(time);
		mirror = cycle % 2 != 0;
	}
	return mirror;
}

void cMotion::MirrorFrame(tFrame& out_frame) const
{
	assert(mParams.mMirrorFunc != nullptr);
	assert(mParams.mLeftJoints.size() > 0 && mParams.mRightJoints.size() > 0);
	mParams.mMirrorFunc(&mParams.mLeftJoints, &mParams.mRightJoints, &out_frame);
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

	fprintf(file, "\"%s\": %.5f,\n", gVelFilterCutoffKey.c_str(), mVelFilterCutoff);

	if (mParams.mRightJoints.size() > 0 && mParams.mLeftJoints.size() > 0)
	{
		fprintf(file, "\"%s\": [", gRightJointsKey.c_str());
		for (size_t i = 0; i < mParams.mRightJoints.size(); ++i)
		{
			if (i != 0)
			{
				fprintf(file, ", ");
			}
			fprintf(file, "%i", mParams.mRightJoints[i]);
		}
		fprintf(file, "],\n");

		fprintf(file, "\"%s\": [", gLeftJointsKey.c_str());
		for (size_t i = 0; i < mParams.mLeftJoints.size(); ++i)
		{
			if (i != 0)
			{
				fprintf(file, ", ");
			}
			fprintf(file, "%i", mParams.mLeftJoints[i]);
		}
		fprintf(file, "],\n");
	}

	fprintf(file, "\"Frames\":\n[\n");

	int num_frames = GetNumFrames();
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
		curr_frame[eFrameTime] = dur;
		std::string frame_json = cJsonUtil::BuildVectorJson(curr_frame);
		fprintf(file, "%s", frame_json.c_str());
	}

	fprintf(file, "\n]");
	fprintf(file, "\n}");
	cFileUtil::CloseFile(file);
}