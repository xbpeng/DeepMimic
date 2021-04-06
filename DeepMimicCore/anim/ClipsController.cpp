#include "anim/ClipsController.h"
#include "anim/KinCharacter.h"

const std::string gMotionsKey = "Motions";
const std::string gFileKey = "File";
const std::string gWeightKey = "Weight";

cClipsController::tMotionEntry::tMotionEntry()
{
	mWeight = 1.0;
}

cClipsController::cClipsController()
{
	mCurrMotionID = gInvalidIdx;
	mEnableRandMotions = true;
}

cClipsController::~cClipsController()
{
}

void cClipsController::Init(cKinCharacter* character, const std::string& param_file)
{
	cMotionController::Init(character, param_file);
	assert(GetNumMotions() > 0);

	int motion_id = 0;
	if (mEnableRandMotions)
	{
		motion_id = SelectNewMotion();
	}
	ActivateMotion(motion_id);
}

void cClipsController::Reset()
{
	cMotionController::Reset();

	if (mEnableRandMotions)
	{
		int motion_id = SelectNewMotion();
		ActivateMotion(motion_id);
	}
}

int cClipsController::GetCurrMotionID() const
{
	return mCurrMotionID;
}

const cMotion& cClipsController::GetMotion() const
{
	int motion_id = GetCurrMotionID();
	return GetMotion(motion_id);
}

cMotion& cClipsController::GetMotion()
{
	int motion_id = GetCurrMotionID();
	return GetMotion(motion_id);
}

const cMotion& cClipsController::GetMotion(int m) const
{
	return mMotions[m].mMotion;
}

cMotion& cClipsController::GetMotion(int m)
{
	return mMotions[m].mMotion;
}

int cClipsController::GetNumMotions() const
{
	return static_cast<int>(mMotions.size());
}

void cClipsController::ChangeMotionDuration(double dur)
{
	for (int i = 0; i < GetNumMotions(); ++i)
	{
		cMotion& motion = GetMotion(i);
		motion.ChangeDuration(dur);
	}
}

double cClipsController::GetMotionWeight(int m) const
{
	return mMotions[m].mWeight;
}

void cClipsController::EnableRandMotions(bool enable)
{
	mEnableRandMotions = enable;
}

void cClipsController::ChangeMotion(int motion_id)
{
	ActivateMotion(motion_id);
}

int cClipsController::SampleMotionID() const
{
	int rand_id = SelectNewMotion();
	return rand_id;
}

int cClipsController::SampleMotionID(const Eigen::VectorXd& clips_cdf) const
{
	int rand_id = SelectNewMotion(clips_cdf);
	return rand_id;
}

bool cClipsController::LoadParams(const std::string& param_file)
{
	bool succ = true;
	if (param_file != "")
	{
		std::ifstream f_stream(param_file);
		Json::Reader reader;
		Json::Value root;
		succ = reader.parse(f_stream, root);
		f_stream.close();

		if (succ)
		{
			if (root[gMotionsKey].isNull())
			{
				succ = false;
			}
			else
			{
				succ = LoadMotions(root[gMotionsKey]);

				if (succ)
				{
					double total_motion_dur = CalcTotalMotionDur();
					printf("Loaded %.3f seconds of motion data.\n", total_motion_dur);

					BuildClipsCDF();
				}
			}
		}
	}

	if (!succ)
	{
		printf("Failed to load clips controller parameters from file %s.\n", param_file.c_str());
	}
	return succ;
}

bool cClipsController::LoadMotions(const Json::Value& json)
{
	bool succ = true;
	bool is_array = json.isArray();
	succ &= is_array;

	if (is_array)
	{
		int num_files = json.size();
		mMotions.reserve(num_files);

		for (int i = 0; i < num_files; ++i)
		{
			Json::Value entry_json = json.get(i, 0);
			std::string curr_file = entry_json.get(gFileKey, "").asString();
			double curr_weight = entry_json.get(gWeightKey, 1.0).asDouble();

			cMotion curr_motion;
			succ &= LoadMotion(curr_file, curr_motion);

			tMotionEntry entry;
			entry.mMotion = curr_motion;
			entry.mWeight = curr_weight;

			if (succ)
			{
				mMotions.push_back(entry);
			}
			else
			{
				printf("Failed to load motion from %s\n", curr_file.c_str());
				assert(false);
			}
		}
	}
	return succ;
}

double cClipsController::CalcTotalMotionDur() const
{
	double total_dur = 0.0;
	for (int i = 0; i < GetNumMotions(); ++i)
	{
		const auto& motion = GetMotion(i);
		double curr_dur = motion.GetDuration();
		total_dur += curr_dur;
	}
	return total_dur;
}

void cClipsController::BuildClipsCDF()
{
	int num_motions = GetNumMotions();
	mClipsCDF.resize(num_motions);
	
	double weight_sum = 0.0;
	for (int m = 0; m < num_motions; ++m)
	{
		const tMotionEntry& entry = mMotions[m];
		double curr_weight = entry.mWeight;

		weight_sum += curr_weight;
		mClipsCDF[m] = weight_sum;
	}

	mClipsCDF /= weight_sum;
}

void cClipsController::ResetParams()
{
	cMotionController::ResetParams();
}

int cClipsController::SelectNewMotion() const
{
	int rand_id = SelectNewMotion(mClipsCDF);
	return rand_id;
}

int cClipsController::SelectNewMotion(const Eigen::VectorXd& clips_cdf) const
{
	assert(clips_cdf.size() > 0);
	assert(clips_cdf.size() == mMotions.size());

	double rand_val = cMathUtil::RandDouble(0.0, 1.0);
	auto it = std::upper_bound(clips_cdf.data(), clips_cdf.data() + clips_cdf.size(), rand_val);
	int rand_id = it - clips_cdf.data();

	return rand_id;
}

void cClipsController::ActivateMotion(int motion_id)
{
	mCurrMotionID = motion_id;
	mCycleRootDelta = CalcCycleRootDelta(GetMotion());
}