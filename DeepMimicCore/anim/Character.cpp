#include "Character.h"
#include <assert.h>
#include "util/json/json.h"

#include "util/FileUtil.h"
#include "util/JsonUtil.h"
#include "render/MeshUtil.h"
#include "render/DrawUtil.h"

// Json keys
const std::string cCharacter::gSkeletonKey = "Skeleton";
const std::string gPoseKey = "Pose";
const std::string gVelKey = "Vel";
const std::string gMeshesKey = "Meshes";

cCharacter::cCharacter()
{
	mID = gInvalidIdx;
	ResetParams();
}

cCharacter::~cCharacter()
{
}

void cCharacter::SetID(int id)
{
	mID = id;
}

int cCharacter::GetID() const
{
	return mID;
}

bool cCharacter::Init(const std::string& char_file, bool load_draw_shapes)
{
	Clear();

	bool succ = true;
	if (char_file != "")
	{
		std::ifstream f_stream(char_file);
		Json::Reader reader;
		Json::Value root;
		succ = reader.parse(f_stream, root);
		f_stream.close();

		if (succ)
		{
			if (root[gSkeletonKey].isNull())
			{
				succ = false;
			}
			else
			{
				succ = LoadSkeleton(root[gSkeletonKey]);
			}
		}
	}

	if (succ)
	{
		InitDefaultState();
		RecordEndEffectors(mEndEffectors);

		if (cDrawUtil::EnableDraw() && load_draw_shapes)
		{
			succ &= LoadMeshes(char_file, mMeshes);
			succ &= LoadDrawShapeDefs(char_file, mDrawShapeDefs);
		}
	}

	if (!succ)
	{
		printf("Failed to parse character from file %s.\n", char_file.c_str());
	}

	return succ;
}

void cCharacter::Clear()
{
	ResetParams();
	mPose.resize(0);
	mVel.resize(0);
	mPose0.resize(0);
	mVel0.resize(0);

	mDrawShapeDefs.resize(0, 0);
	mMeshes.clear();
}

void cCharacter::Update(double time_step)
{
}

void cCharacter::Reset()
{
	ResetParams();

	const Eigen::VectorXd& pose0 = GetPose0();
	const Eigen::VectorXd& vel0 = GetVel0();
	
	SetPose(pose0);
	SetVel(vel0);
}

int cCharacter::GetNumDof() const
{
	int dofs = cKinTree::GetNumDof(mJointMat);
	return dofs;
}

const Eigen::MatrixXd& cCharacter::GetJointMat() const
{
	return mJointMat;
}

int cCharacter::GetNumJoints() const
{
	return cKinTree::GetNumJoints(mJointMat);
}

cKinTree::eJointType cCharacter::GetJointType(int joint_id) const
{
	return cKinTree::GetJointType(mJointMat, joint_id);
}

const Eigen::VectorXd& cCharacter::GetPose() const
{
	return mPose;
}

void cCharacter::SetPose(const Eigen::VectorXd& pose)
{
	assert(pose.size() == GetNumDof());
	mPose = pose;
}

const Eigen::VectorXd& cCharacter::GetPose0() const
{
	return mPose0;
}

void cCharacter::SetPose0(const Eigen::VectorXd& pose)
{
	mPose0 = pose;
}

const Eigen::VectorXd& cCharacter::GetVel() const
{
	return mVel;
}

void cCharacter::SetVel(const Eigen::VectorXd& vel)
{
	assert(vel.size() == GetNumDof());
	mVel = vel;
}

const Eigen::VectorXd& cCharacter::GetVel0() const
{
	return mVel0;
}

void cCharacter::SetVel0(const Eigen::VectorXd& vel)
{
	mVel0 = vel;
}

int cCharacter::GetRootID() const
{
	int root_id = cKinTree::GetRootID();
	return root_id;
}

tVector cCharacter::GetRootPos() const
{
	tVector pos = cKinTree::GetRootPos(mPose);
	return pos;
}

void cCharacter::GetRootRotation(tVector& out_axis, double& out_theta) const
{
	tQuaternion quat = GetRootRotation();
	cMathUtil::QuaternionToAxisAngle(quat, out_axis, out_theta);
}

tQuaternion cCharacter::GetRootRotation() const
{
	return cKinTree::GetRootRot(mPose);
}

void cCharacter::SetRootPos(const tVector& pos)
{
	cKinTree::SetRootPos(pos, mPose);
}

void cCharacter::SetRootPos0(const tVector& pos)
{
	cKinTree::SetRootPos(pos, mPose0);
}

void cCharacter::SetRootRotation(const tQuaternion& q)
{
	cKinTree::SetRootRot(q, mPose);
}

void cCharacter::RotateRoot(const tQuaternion& rot)
{
	tQuaternion root_rot = GetRootRotation();
	root_rot = rot * root_rot;
	root_rot.normalize();
	SetRootRotation(root_rot);
}

void cCharacter::SetRootVel(const tVector& vel)
{
	cKinTree::SetRootVel(vel, mVel);
}

void cCharacter::SetRootAngVel(const tVector& ang_vel)
{
	cKinTree::SetRootAngVel(ang_vel, mVel);
}

tQuaternion cCharacter::CalcHeadingRot() const
{
	return cKinTree::CalcHeadingRot(mPose);
}

double cCharacter::CalcHeading() const
{
	return cKinTree::CalcHeading(mPose);
}

tMatrix cCharacter::BuildOriginTrans() const
{
	return cKinTree::BuildOriginTrans(mPose);
}

int cCharacter::GetParamOffset(int joint_id) const
{
	return cKinTree::GetParamOffset(mJointMat, joint_id);
}

int cCharacter::GetParamSize(int joint_id) const
{
	return cKinTree::GetParamSize(mJointMat, joint_id);
}

bool cCharacter::IsEndEffector(int joint_id) const
{
	return cKinTree::IsEndEffector(mJointMat, joint_id);
}

int cCharacter::GetParentJoint(int joint_id) const
{
	return cKinTree::GetParent(mJointMat, joint_id);
}

tVector cCharacter::CalcJointPos(int joint_id) const
{
	tVector pos = cKinTree::CalcJointWorldPos(mJointMat, mPose, joint_id);
	return pos;
}

tVector cCharacter::CalcJointVel(int joint_id) const
{
	tVector pos = cKinTree::CalcJointWorldVel(mJointMat, mPose, mVel, joint_id);
	return pos;
}

void cCharacter::CalcJointWorldRotation(int joint_id, tVector& out_axis, double& out_theta) const
{
	cKinTree::CalcJointWorldTheta(mJointMat, mPose, joint_id, out_axis, out_theta);
}

tQuaternion cCharacter::CalcJointWorldRotation(int joint_id) const
{
	tVector axis;
	double theta;
	CalcJointWorldRotation(joint_id, axis, theta);
	return cMathUtil::AxisAngleToQuaternion(axis, theta);
}

double cCharacter::CalcJointChainLength(int joint_id)
{
	auto chain = cKinTree::FindJointChain(mJointMat, GetRootID(), joint_id);
	return cKinTree::CalcChainLength(mJointMat, chain);
}

tMatrix cCharacter::BuildJointWorldTrans(int joint_id) const
{
	return cKinTree::JointWorldTrans(mJointMat, mPose, joint_id);
}

void cCharacter::CalcAABB(tVector& out_min, tVector& out_max) const
{
	cKinTree::CalcAABB(mJointMat, mPose, out_min, out_max);
}

int cCharacter::GetNumEndEffectors() const
{
	return static_cast<int>(mEndEffectors.size());
}

const Eigen::VectorXi& cCharacter::GetEndEffectors() const
{
	return mEndEffectors;
}

// weights for each joint used to compute the pose error during training
double cCharacter::GetJointDiffWeight(int joint_id) const
{
	return cKinTree::GetJointDiffWeight(mJointMat, joint_id);
}

bool cCharacter::WriteState(const std::string& file) const
{
	return WriteState(file, tMatrix::Identity());
}

bool cCharacter::WriteState(const std::string& file, const tMatrix& root_trans) const
{
	Eigen::VectorXd pose = GetPose();
	Eigen::VectorXd vel = GetVel();

	tQuaternion trans_q = cMathUtil::RotMatToQuaternion(root_trans);

	tVector root_pos = cKinTree::GetRootPos(pose);
	tQuaternion root_rot = cKinTree::GetRootRot(pose);
	tVector root_vel = cKinTree::GetRootVel(vel);
	tVector root_ang_vel = cKinTree::GetRootAngVel(vel);

	root_pos[3] = 1;
	root_pos = root_trans * root_pos;
	root_pos[3] = 0;

	root_rot = trans_q * root_rot;
	root_vel = root_trans * root_vel;
	root_ang_vel = root_trans * root_ang_vel;

	cKinTree::SetRootPos(root_pos, pose);
	cKinTree::SetRootRot(root_rot, pose);
	cKinTree::SetRootVel(root_vel, vel);
	cKinTree::SetRootAngVel(root_ang_vel, vel);

	std::string json = BuildStateJson(pose, vel);
	FILE* f = cFileUtil::OpenFile(file, "w");
	if (f != nullptr)
	{
		fprintf(f, "%s", json.c_str());
		cFileUtil::CloseFile(f);
		return true;
	}
	return false;
}

bool cCharacter::ReadState(const std::string& file)
{
	std::ifstream f_stream(file);
	Json::Reader reader;
	Json::Value root;
	bool succ = reader.parse(f_stream, root);
	f_stream.close();

	if (succ && !root[gPoseKey].isNull())
	{
		Eigen::VectorXd pose;
		succ &= ParseState(root[gPoseKey], pose);
		cKinTree::PostProcessPose(mJointMat, pose);
		SetPose(pose);
	}

	if (succ && !root[gVelKey].isNull())
	{
		Eigen::VectorXd vel;
		succ &= ParseState(root[gVelKey], vel);
		SetVel(vel);
	}

	return succ;
}

bool cCharacter::LoadSkeleton(const Json::Value& root)
{
	return cKinTree::Load(root, mJointMat);
}

void cCharacter::InitDefaultState()
{
	int state_size = GetNumDof();
	cKinTree::BuildDefaultPose(mJointMat, mPose0);
	cKinTree::BuildDefaultVel(mJointMat, mVel0);
	mPose = mPose0;
	mVel = mVel0;
}

bool cCharacter::HasDrawShapes() const
{
	return mDrawShapeDefs.size() > 0;
}

const Eigen::MatrixXd& cCharacter::GetDrawShapeDefs() const
{
	return mDrawShapeDefs;
}

const std::shared_ptr<cDrawMesh>& cCharacter::GetMesh(int i) const
{
	assert(i >= 0 && i < GetNumMeshes());
	return mMeshes[i];
}

int cCharacter::GetNumMeshes() const
{
	return static_cast<int>(mMeshes.size());
}

void cCharacter::ResetParams()
{
}

bool cCharacter::ParseState(const Json::Value& root, Eigen::VectorXd& out_state) const
{
	bool succ = cJsonUtil::ReadVectorJson(root, out_state);
	int num_dof = GetNumDof();
	assert(out_state.size() == num_dof);
	return succ;
}

std::string cCharacter::BuildStateJson(const Eigen::VectorXd& pose, const Eigen::VectorXd& vel) const
{
	std::string json = "";

	std::string pose_json = cJsonUtil::BuildVectorJson(pose);
	std::string vel_json = cJsonUtil::BuildVectorJson(vel);

	json = "{\n\"Pose\":" + pose_json + ",\n\"Vel\":" + vel_json + "\n}";
	return json;
}


bool cCharacter::LoadDrawShapeDefs(const std::string& char_file, Eigen::MatrixXd& out_draw_defs) const
{
	bool succ = cKinTree::LoadDrawShapeDefs(char_file, out_draw_defs);
	return succ;
}

bool cCharacter::LoadMeshes(const std::string& char_file, std::vector<std::shared_ptr<cDrawMesh>>& out_meshes) const
{
	bool succ = true;
	std::string str;

	std::ifstream f_stream(char_file.c_str());
	Json::Value root;
	Json::Reader reader;
	succ = reader.parse(f_stream, root);
	f_stream.close();

	if (succ)
	{
		if (!root[gMeshesKey].isNull())
		{
			Json::Value meshes = root.get(gMeshesKey, 0);
			int num_meshes = meshes.size();

			succ = true;
			out_meshes.resize(num_meshes);
			for (int i = 0; i < num_meshes; ++i)
			{
				std::string curr_path = meshes.get(i, "").asString();
				std::shared_ptr<cDrawMesh> curr_mesh = std::shared_ptr<cDrawMesh>(new cDrawMesh);
				succ &= cMeshUtil::LoadObj(curr_path, *curr_mesh);
				if (succ)
				{
					out_meshes[i] = curr_mesh;
				}
				else
				{
					printf("Failed to load mesh from %s\n", curr_path.c_str());
					assert(false);
					break;
				}
			}
		}
	}

	if (!succ)
	{
		printf("Failed to load meshes from %s\n", char_file.c_str());
		assert(false);
	}

	return succ;
}

void cCharacter::RecordEndEffectors(Eigen::VectorXi& out_end_effs) const
{
	std::vector<int> effs;
	for (int j = 0; j < GetNumJoints(); ++j)
	{
		if (IsEndEffector(j))
		{
			effs.push_back(j);
		}
	}

	out_end_effs.resize(effs.size());
	for (size_t i = 0; i < effs.size(); ++i)
	{
		int joint_id = effs[i];
		out_end_effs[i] = joint_id;
	}
}