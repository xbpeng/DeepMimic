#include "anim/MotionController.h"
#include "anim/KinCharacter.h"

cMotionController::cMotionController()
{
	mCycleRootDelta.setZero();
}

cMotionController::~cMotionController()
{
}

void cMotionController::Init(cKinCharacter* character, const std::string& param_file)
{
	cKinController::Init(character, param_file);
	LoadParams(param_file);
}

void cMotionController::Clear()
{
	cKinController::Clear();
	mCycleRootDelta.setZero();
}

void cMotionController::CalcPose(double time, Eigen::VectorXd& out_pose) const
{
	const cMotion& motion = GetMotion();
	motion.CalcFrame(time, out_pose);

	if (EnableMotionLoop())
	{
		tVector root_offset;
		CalcRootCycleOffset(time, root_offset);

		const auto& joint_mat = mChar->GetJointMat();
		tVector root_pos = cKinTree::GetRootPos(out_pose);
		root_pos += root_offset;

		cKinTree::SetRootPos(root_pos, out_pose);
	}
}

void cMotionController::CalcVel(double time, Eigen::VectorXd& out_vel) const
{
	const cMotion& motion = GetMotion();
	motion.CalcFrameVel(time, out_vel);
}

const cMotion& cMotionController::GetMotion() const
{
	return mMotion;
}

cMotion& cMotionController::GetMotion()
{
	return mMotion;
}

int cMotionController::GetCycle() const
{
	int cycle = 0;
	if (EnableMotionLoop())
	{
		double phase = mTime / GetMotionDuration();
		cycle = static_cast<int>(std::floor(phase));
	}
	return cycle;
}

double cMotionController::GetPhase() const
{
	const cMotion& motion = GetMotion();
	double phase = motion.CalcPhase(mTime);
	return phase;
}

void cMotionController::ChangeMotionDuration(double dur)
{
	cMotion& motion = GetMotion();
	motion.ChangeDuration(dur);
}

tVector cMotionController::GetCycleRootDelta() const
{
	return mCycleRootDelta;
}

bool cMotionController::LoadParams(const std::string& param_file)
{
	bool succ = LoadMotion(param_file, mMotion);
	if (succ)
	{
		mCycleRootDelta = CalcCycleRootDelta(GetMotion());
	}
	else
	{
		printf("Failed to load motion from %s\n", param_file.c_str());
	}
	return succ;
}

void cMotionController::CalcRootCycleOffset(double time, tVector& out_offset) const
{
	const cMotion& motion = GetMotion();
	tVector cycle_delta = GetCycleRootDelta();
	tVector root_delta = tVector::Zero();

	int cycle_count = motion.CalcCycleCount(time);
	root_delta = cycle_count * cycle_delta;
	out_offset = root_delta;
}
