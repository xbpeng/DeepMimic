#include "anim/KinController.h"
#include "anim/KinCharacter.h"
#include "util/FileUtil.h"

cKinController::cKinController()
{
	mTime = 0;
}

cKinController::~cKinController()
{
}

void cKinController::Init(cKinCharacter* character, const std::string& param_file)
{
	assert(character != nullptr);
	Clear();
	mChar = character;
}

void cKinController::Reset()
{
	ResetParams();
}

void cKinController::Clear()
{
	ResetParams();
	mChar = nullptr;
}

void cKinController::Update(double time_step)
{
	mTime += time_step;
}

void cKinController::SetTime(double time)
{
	mTime = time;
}

double cKinController::GetTime() const
{
	return mTime;
}

double cKinController::GetMotionDuration() const
{
	const cMotion& motion = GetMotion();
	return motion.GetDuration();
}

int cKinController::GetNumMotionFrames() const
{
	const cMotion& motion = GetMotion();
	return motion.GetNumFrames();
}

cMotion::tFrame cKinController::GetMotionFrame(int f) const
{
	const cMotion& motion = GetMotion();
	return motion.GetFrame(f);
}

cMotion::tFrame cKinController::GetMotionFrameVel(int f) const
{
	const cMotion& motion = GetMotion();
	return motion.GetFrameVel(f);
}

bool cKinController::EnableMotionLoop() const
{
	const cMotion& motion = GetMotion();
	return motion.EnableLoop();
}

bool cKinController::IsMotionOver() const
{
	const cMotion& motion = GetMotion();
	return motion.IsOver(mTime);
}

void cKinController::ResetParams()
{
	mTime = 0;
}

void cKinController::BuildMotionParams(const std::string& motion_file, cKinCharacter& kin_char,
										cMotion::tParams& out_params) const
{
	out_params.mMotionFile = motion_file;

	out_params.mBlendFunc = std::bind(&cKinCharacter::BlendFrames, &kin_char,
		std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
		std::placeholders::_4);

	out_params.mVelFunc = std::bind(&cKinCharacter::CalcFrameVel, &kin_char,
		std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
		std::placeholders::_4);

	out_params.mPostProcessFunc = std::bind(&cKinCharacter::PostProcessPose, &kin_char, std::placeholders::_1);
}

bool cKinController::LoadMotion(const std::string& motion_file, cMotion& out_motion) const
{
	cMotion::tParams motion_params;
	BuildMotionParams(motion_file, *mChar, motion_params);

	bool succ = out_motion.Load(motion_params);

	if (succ)
	{
		int char_dof = mChar->GetNumDof();
		int motion_dof = out_motion.GetNumDof();
		if (char_dof != motion_dof)
		{
			printf("DOF mismatch, char dof: %i, motion dof: %i\n", char_dof, motion_dof);
			succ = false;
			assert(false);
		}

		PostProcessMotion(out_motion);
	}
	else
	{
		assert(false);
	}
	return succ;
}

void cKinController::PostProcessMotion(cMotion& out_motion) const
{
	const auto& joint_mat = mChar->GetJointMat();
	Eigen::VectorXd frame_beg = out_motion.GetFrame(0);
	tVector root_pos_beg = cKinTree::GetRootPos(frame_beg);

	int num_frames = out_motion.GetNumFrames();
	for (int f = 0; f < num_frames; ++f)
	{
		Eigen::VectorXd frame = out_motion.GetFrame(f);
		tVector root_pos = cKinTree::GetRootPos(frame);
		root_pos[0] -= root_pos_beg[0];
		root_pos[2] -= root_pos_beg[2];
		cKinTree::SetRootPos(root_pos, frame);
		out_motion.SetFrame(f, frame);
	}
}

tVector cKinController::CalcCycleRootDelta(const cMotion& motion) const
{
	const auto& joint_mat = mChar->GetJointMat();
	int num_frames = motion.GetNumFrames();
	Eigen::VectorXd frame_beg = motion.GetFrame(0);
	Eigen::VectorXd  frame_end = motion.GetFrame(num_frames - 1);

	tVector root_pos_beg = cKinTree::GetRootPos(frame_beg);
	tVector root_pos_end = cKinTree::GetRootPos(frame_end);

	tVector delta = root_pos_end - root_pos_beg;
	delta[1] = 0;
	return delta;
}