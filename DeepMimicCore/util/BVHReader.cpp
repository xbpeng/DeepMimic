/*
 * BVHReader.cpp
 *
 *  Created on: Nov 4, 2016
 *      Author: Glen
 */

#include "BVHReader.h"
#include <iostream>
#include <deque>
#include <stack>

const std::string cBVHReader::gReserved[] = { "HIERARCHY", "ROOT", "OFFSET", "CHANNELS", "MOTION" };
const std::string cBVHReader::gChannelNames[] = { "Xposition", "Yposition", "Zposition", "Zrotation", "Xrotation", "Yrotation" };

//const double gScale = gInchesToMeters;
const double gScale = 0.01;
//const double gScale = 0.004; // dragon
//const double gScale = 0.003; // t-rex

int cBVHReader::tJointData::CalcPoseDim() const
{
	int dim = 0;
	if (IsRoot())
	{
		dim = cKinTree::gRootDim;
	}
	else if (IsValid())
	{
		dim = cKinTree::GetJointParamSize(mJointType);
	}
	return dim;
}

bool cBVHReader::tJointData::IsRoot() const
{
	return parent == gInvalidIdx;
}

bool cBVHReader::tJointData::IsValid() const
{
	// end effectors don't count as joints
	bool valid = (num_channels > 0) || (children.size() > 0);
	return valid;
}

void cBVHReader::PoseData::Clear()
{
	channels_types.clear();
	data.resize(0, 0);
}

cBVHReader::cBVHReader()
{
	mModelTrans.setIdentity();
	mChannelCounter = 0;
}

cBVHReader::~cBVHReader() {
	// TODO Auto-generated destructor stub
}

void cBVHReader::printData() const
{
	const auto& root = GetRootJoint();
	printJointHeirachy(root);

	std::cout << "motionData: " << motionData.data.row(1) << std::endl;
};

void cBVHReader::printJointHeirachy(const tJointData& joint) const
{
	std::cout << "Joint Name: " << joint.name << std::endl;
	for (auto ct = joint.children.begin(); ct != joint.children.end(); ++ct)
	{
		int child_id = *ct;
		const tJointData& child_joint = GetJointData(child_id);
		if (child_joint.children.size() > 0)
		{
			this->printJointHeirachy(child_joint);
		}
	}
}

void cBVHReader::parseBVH(const std::string& filename)
{
	Clear();

	std::fstream file;
	file.open(filename.c_str(), std::ios_base::in);

	if (file.is_open())
	{
		std::string line;

		while (file.good())
		{
			file >> line;
			if (trim(line) == "HIERARCHY")
			{
				parseHierarchy(file);
			}
			break;
		}

		file.close();
	}
	else
	{
		std::cerr << "Error opening BVN file " << filename << std::endl;
	}
}

void cBVHReader::SetModelTransform(const tMatrix& trans)
{
	mModelTrans = trans;
}

void cBVHReader::parseHierarchy(std::istream& stream)
{
	std::string tmp;

	while (stream.good())
	{
		stream >> tmp;

		if (trim(tmp) == "ROOT")
		{
			parseJoint(stream);
		}
		else if (trim(tmp) == "MOTION")
		{
			parseMotion(stream);
		}
	}
}


void cBVHReader::parseMotion(std::istream& stream)
{
	std::string tmp;

	while (stream.good())
	{
		stream >> tmp;

		if (trim(tmp) == "Frames:")
		{
			stream >> motionData.num_frames;
		}
		else if (trim(tmp) == "Time:")
		{
			float frame_step;
			stream >> frame_step;

			motionData.frame_step = frame_step;
			size_t num_frames = motionData.num_frames;
			size_t num_channels = motionData.num_motion_channels;

			motionData.data = Eigen::MatrixXd::Ones( num_frames , num_channels);

			for (size_t frame = 0; frame < num_frames; frame++)
			{
				for (int channel = 0; channel < num_channels; channel++)
				{
					double x;
					std::stringstream ss;
					stream >> tmp;
					ss << tmp;
					ss >> x;
					if ((motionData.channels_types[channel] == Xrotation) ||
						(motionData.channels_types[channel] == Yrotation) ||
						(motionData.channels_types[channel] == Zrotation))
					{  // Need to convert degrees to radians from these files.
						x *= gDegreesToRadians;
					}
					else
					{
						x *= gScale;
					}
					motionData.data(frame, channel) = x;
				}
			}
		}
	}
}

void cBVHReader::Clear()
{
	mJointData.clear();
	motionData.Clear();
	mChannelCounter = 0;
}

int cBVHReader::parseJoint(std::istream& stream, int parent_id)
{
	int joint_id = AddJoint();
	tJointData joint;
	joint.parent = parent_id;

	//    stats.num_total_joints++;
	//    allJoints.insert(joint);

	// load joint name
	std::string* name = new std::string;
	stream >> *name;
	joint.name = name->c_str();

	std::string tmp;
	joint.matrix = tMatrix::Identity();

	unsigned channel_order_index = 0;
	while (stream.good())
	{
		stream >> tmp;
		tmp = trim(tmp);

		// setting channels
		char c = tmp.at(0);
		if (c == 'X' || c == 'Y' || c == 'Z')
		{
			if (tmp == "Xposition")
			{
				joint.channels_order.push_back(Xposition);
				motionData.channels_types.push_back(Xposition);
			}
			else if (tmp == "Yposition")
			{
				joint.channels_order.push_back(Yposition);
				motionData.channels_types.push_back(Yposition);
			}
			else if (tmp == "Zposition")
			{
				joint.channels_order.push_back(Zposition);
				motionData.channels_types.push_back(Zposition);
			}
			else if (tmp == "Xrotation")
			{
				joint.channels_order.push_back(Xrotation);
				motionData.channels_types.push_back(Xrotation);
			}
			else if (tmp == "Yrotation")
			{
				joint.channels_order.push_back(Yrotation);
				motionData.channels_types.push_back(Yrotation);
			}
			else if (tmp == "Zrotation")
			{
				joint.channels_order.push_back(Zrotation);
				motionData.channels_types.push_back(Zrotation);
			}
		}

		if (tmp == "OFFSET")
		{
			stream >> joint.offset(0)
				>> joint.offset(1)
				>> joint.offset(2);
			joint.offset *= gScale;
			joint.offset = mModelTrans * joint.offset;
			tMatrix trans = cMathUtil::TranslateMat(joint.offset);
			joint.matrix = trans;
		}
		else if (tmp == "CHANNELS")
		{
			// loading num of channels
			stream >> joint.num_channels;

			// adding to statistics
			motionData.num_motion_channels += joint.num_channels;

			// increasing static counter of channel index startin motion section
			joint.channel_start = mChannelCounter;
			mChannelCounter += joint.num_channels;

			// creating array for channel order specification
			// joint->channels_order = new short[joint->num_channels];

		}
		else if (tmp == "JOINT")
		{
			int child_id = parseJoint(stream, joint_id);
			tJointData& child_joint = GetJointData(child_id);
			joint.children.push_back(child_id);
		}
		else if (tmp == "End")
		{
			// End Site {
			stream >> tmp >> tmp;

			int child_id = AddJoint();
			tJointData& end_joint = GetJointData(child_id);
			end_joint.parent = joint_id;
			end_joint.num_channels = 0;
			end_joint.name = "EndSite";
			end_joint.mJointType = cKinTree::eJointTypeFixed;
			joint.children.push_back(child_id);

			//            allJoints.insert(tmp_joint);

			stream >> tmp;
			if (tmp == "OFFSET")
			{
				stream >> end_joint.offset(0)
						>> end_joint.offset(1)
						>> end_joint.offset(2);
				end_joint.offset *= gScale;
				end_joint.offset = mModelTrans * end_joint.offset;
				tMatrix trans = cMathUtil::TranslateMat(end_joint.offset);
				end_joint.matrix = trans;
			}
			stream >> tmp;
		}
		else if (tmp == "}")
		{
			break;
		}
	}

	joint.mJointType = CalcJointType(joint);
	mJointData[joint_id] = joint;
	return joint_id;
}

tMatrix cBVHReader::getTransformationForFrame(const tJointData& joint, size_t frame_) const
{
	return this->getTransformationForFrame(joint.channel_start, joint.channels_order, motionData.data, frame_);
}

tMatrix cBVHReader::getTranslationForFrame(const tJointData& joint, size_t frame_) const
{
	return this->getTranslationForFrame(joint.channel_start, joint.channels_order, motionData.data, frame_);
}

tMatrix cBVHReader::getRotationForFrame(const tJointData& joint, size_t frame_) const
{
	return this->getRotationForFrame(joint.channel_start, joint.channels_order, motionData.data, frame_);
}

const cBVHReader::tJointData& cBVHReader::GetRootJoint() const
{
	return GetJointData(0);
}

const cBVHReader::tJointData& cBVHReader::GetJointData(int joint_id) const
{
	assert(joint_id >= 0 && joint_id < mJointData.size());
	return mJointData[joint_id];
}

cBVHReader::tJointData& cBVHReader::GetJointData(int joint_id)
{
	assert(joint_id >= 0 && joint_id < mJointData.size());
	return mJointData[joint_id];
}

int cBVHReader::FindJoint(std::string joint_name) const
{
	int joint_id = gInvalidIdx;

	for (int j = 0; j < GetNumJoints(); ++j)
	{
		const tJointData& joint_data = GetJointData(j);
		if (joint_data.name == joint_name)
		{
			joint_id = j;
			break;
		}
	}

	return joint_id;
}

int cBVHReader::GetNumJoints() const
{
	return static_cast<int>(mJointData.size());
}

void cBVHReader::BuildJointMat(Eigen::MatrixXd& out_joint_mat) const
{
	std::vector<int> valid_joints;
	FetchValidJoints(valid_joints);
	
	int num_joints = static_cast<int>(valid_joints.size());
	std::vector<int> joint_id_to_idx(GetNumJoints());
	for (size_t j = 0; j < num_joints; ++j)
	{
		int joint_id = valid_joints[j];
		joint_id_to_idx[joint_id] = static_cast<int>(j);
	}
	
	out_joint_mat.resize(num_joints, cKinTree::eJointDescMax);
	for (int j = 0; j < num_joints; ++j)
	{
		int joint_id = valid_joints[j];
		const tJointData& joint = GetJointData(joint_id);
		int parent_idx = gInvalidIdx;
		if (joint.parent != gInvalidIdx)
		{
			parent_idx = joint_id_to_idx[joint.parent];
		}
		assert(parent_idx < j);

		bool is_end = true;
		for (size_t c = 0; c < joint.children.size(); ++c)
		{
			const tJointData& child_joint = GetJointData(joint.children[c]);
			if (child_joint.IsValid())
			{
				is_end = false;
				break;
			}
		}

		cKinTree::tJointDesc curr_desc = cKinTree::BuildJointDesc();
		curr_desc[cKinTree::eJointDescType] = static_cast<int>(joint.mJointType);
		curr_desc[cKinTree::eJointDescParent] = parent_idx;
		curr_desc[cKinTree::eJointDescAttachX] = joint.offset[0];
		curr_desc[cKinTree::eJointDescAttachY] = joint.offset[1];
		curr_desc[cKinTree::eJointDescAttachZ] = joint.offset[2];
		curr_desc[cKinTree::eJointDescIsEndEffector] = (is_end) ? 1 : 0;

		out_joint_mat.row(j) = curr_desc;
	}
}

void cBVHReader::BuildMotion(double target_framerate, cMotion& out_motion) const
{
	int frame_inc = 1;
	if (target_framerate > 0)
	{
		double target_frame_step = 1 / target_framerate;
		frame_inc = static_cast<int>(target_frame_step / motionData.frame_step);
		frame_inc = std::max(frame_inc, 1);
	}

	int pose_dim = CalcPoseDim();
	int num_frames = static_cast<int>(GetNumFrames() / frame_inc);
	out_motion.Init(num_frames, pose_dim);

	double frame_step = frame_inc * motionData.frame_step;
	for (int f = 0; f < num_frames; ++f)
	{
		double curr_time = f * frame_step;
		int frame_idx = f * frame_inc;
		Eigen::VectorXd curr_pose;
		ConvertFrameToPose(frame_idx, curr_pose);

		out_motion.SetFrame(f, curr_pose);
		out_motion.SetFrameTime(f, curr_time);
	}
}

int cBVHReader::CalcPoseDim() const
{
	int num_dofs = 0;
	for (int j = 0; j < GetNumJoints(); ++j)
	{
		const tJointData& joint = GetJointData(j);
		int dim = joint.CalcPoseDim();
		num_dofs += dim;
	}
	return num_dofs;
}

double cBVHReader::GetFramerate() const
{
	return 1 / motionData.frame_step;
}

tMatrix cBVHReader::getTransformationForFrame(size_t channel_start, const std::vector<Channels> & channels_order, const Eigen::MatrixXd & frame_data,
												size_t frame) const
{
	tMatrix trans_mat = getTranslationForFrame(channel_start, channels_order, frame_data, frame);
	tMatrix rot_mat = getRotationForFrame(channel_start, channels_order, frame_data, frame);
	tMatrix mat = trans_mat * rot_mat;
	return mat;
}

tMatrix cBVHReader::getTranslationForFrame(size_t channel_start, const std::vector<Channels> & channels_order, const Eigen::MatrixXd & frame_data,
											size_t frame) const
{
	tMatrix mat = tMatrix::Identity();

	for (size_t i = 0; i < channels_order.size(); i++)
	{
		Channels curr_channel = channels_order[i];
		double val = frame_data(frame, channel_start + i);
		if (curr_channel == Xposition)
		{
			mat(0, 3) = val;
		}
		else if (curr_channel == Yposition)
		{
			mat(1, 3) = val;
		}
		else if (curr_channel == Zposition)
		{
			mat(2, 3) = val;
		}
	}

	mat(3, 3) = 0; // ignore translation for local transforms
	mat = mModelTrans * mat;
	mat(3, 3) = 1;
	return mat;
}

tMatrix cBVHReader::getRotationForFrame(size_t channel_start, const std::vector<Channels> & channels_order, const Eigen::MatrixXd & frame_data,
										size_t frame) const
{
	tMatrix mat = tMatrix::Identity();

	for (size_t i = 0; i < channels_order.size(); i++)
	{
		Channels curr_channel = channels_order[i];
		double val = frame_data(frame, channel_start + i);
		switch (curr_channel)
		{
		case Xrotation:
			mat *= cMathUtil::RotateMat(mModelTrans * tVector(1, 0, 0, 0), val);
			break;
		case Yrotation:
			mat *= cMathUtil::RotateMat(mModelTrans * tVector(0, 1, 0, 0), val);
			break;
		case Zrotation:
			mat *= cMathUtil::RotateMat(mModelTrans * tVector(0, 0, 1, 0), val);
			break;
		default:
			break;
		}
	}

	return mat;
}

bool cBVHReader::GetJointLocation(std::string jointName, size_t frame_, tVector & position,
	tMatrix & transform) const
{
	position = tVector::Zero();
	position(3) = 1.0;

	transform = tMatrix::Identity();
	const tJointData& root_joint = GetRootJoint();
	return GetJointLocation(root_joint, jointName, frame_, position, transform);
}

bool cBVHReader::GetJointLocation(const tJointData& joint, std::string jointName, size_t frame_, tVector & position,
									tMatrix & transform) const
{
	tMatrix transJoint = joint.matrix * getTransformationForFrame(joint.channel_start, joint.channels_order, motionData.data, frame_);
	tMatrix transTMP = transform * transJoint;
	if (joint.name == jointName)
	{
		transform = transTMP;
		position = transform * position;
		return true;
	}
		
	// add children to stack
	for (size_t j = 0; j < joint.children.size(); j++)
	{
		const tJointData& child_data = GetJointData(joint.children[j]);
		bool found = GetJointLocation(child_data, jointName, frame_, position, transTMP);
		if (found)
		{
			transform = transTMP;
			return found;
		}
	}
	
	return false;
}

int cBVHReader::AddJoint()
{
	int joint_id = static_cast<int>(mJointData.size());
	mJointData.push_back(tJointData());
	return joint_id;
}

void cBVHReader::FetchValidJoints(std::vector<int>& out_joints) const
{
	int num_joints = GetNumJoints();
	out_joints.reserve(num_joints);

	for (int j = 0; j < GetNumJoints(); ++j)
	{
		const tJointData& joint = GetJointData(j);
		bool valid = joint.IsValid();
		if (valid)
		{
			out_joints.push_back(j);
		}
	}
}

cKinTree::eJointType cBVHReader::CalcJointType(const tJointData& joint_data) const
{
	cKinTree::eJointType joint_type = cKinTree::eJointTypeNone;
	if (joint_data.parent == gInvalidIdx)
	{
		joint_type = cKinTree::eJointTypeNone;
	}
	else
	{
		unsigned int channel_mask = 0;
		for (size_t i = 0; i < joint_data.channels_order.size(); ++i)
		{
			Channels curr_channel = joint_data.channels_order[i];
			channel_mask |= 1 << curr_channel;
		}

		switch (channel_mask)
		{
		case 0:
			joint_type = cKinTree::eJointTypeFixed;
			break;
		case (1 << Xrotation) :
		case (1 << Yrotation) :
		case (1 << Zrotation) :
			joint_type = cKinTree::eJointTypeRevolute;
			break;
		case (1 << Xposition) :
		case (1 << Yposition) :
		case (1 << Zposition) :
			joint_type = cKinTree::eJointTypePrismatic;
			break;
		case (1 << Xrotation) | (1 << Yrotation) | (1 << Zrotation) :
			joint_type = cKinTree::eJointTypeSpherical;
			break;
		case (1 << Xposition) | (1 << Yposition) :
		case (1 << Xposition) | (1 << Zposition) :
		case (1 << Yposition) | (1 << Zposition) :
			joint_type = cKinTree::eJointTypePlanar;
			break;
		case (1 << Xrotation) | (1 << Yrotation) | (1 << Zrotation) 
			| (1 << Xposition) | (1 << Yposition) | (1 << Zposition) :
			joint_type = cKinTree::eJointTypeSpherical; // hack
			break;
		default:
			assert(false); // unsupported joint type
			break;
		}
	}

	return joint_type;
}

void cBVHReader::ConvertFrameToPose(int f, Eigen::VectorXd& out_pose) const
{
	int pose_dim = CalcPoseDim();
	out_pose.resize(pose_dim);
	int param_offset = 0;
	for (int j = 0; j < GetNumJoints(); ++j)
	{
		const tJointData& joint = GetJointData(j);
		if (joint.IsValid())
		{
			Eigen::VectorXd joint_pose;
			BuildJointPose(j, f, joint_pose);

			int param_size = static_cast<int>(joint_pose.size());
			out_pose.segment(param_offset, param_size) = joint_pose;
			param_offset += param_size;
		}
	}
	assert(param_offset == out_pose.size());
}

void cBVHReader::BuildJointPose(int j, int frame, Eigen::VectorXd& out_pose) const
{
	const tJointData& joint = GetJointData(j);
	
	if (joint.IsRoot())
	{
		BuildJointPoseRoot(j, frame, out_pose);
	}
	else
	{
		switch (joint.mJointType)
		{
		case cKinTree::eJointTypeRevolute:
			BuildJointPoseRevolute(j, frame, out_pose);
			break;
		case cKinTree::eJointTypePlanar:
			BuildJointPosePlanar(j, frame, out_pose);
			break;
		case cKinTree::eJointTypePrismatic:
			BuildJointPosePrismatic(j, frame, out_pose);
			break;
		case cKinTree::eJointTypeFixed:
			BuildJointPoseFixed(j, frame, out_pose);
			break;
		case cKinTree::eJointTypeSpherical:
			BuildJointPoseSpherical(j, frame, out_pose);
			break;
		default:
			assert(false); // unsupported joint type
			break;
		}
	}
}

void cBVHReader::BuildJointPoseRoot(int j, int frame, Eigen::VectorXd& out_pose) const
{
	const tJointData& joint = GetJointData(j);
	int param_size = cKinTree::gRootDim;
	out_pose = Eigen::VectorXd::Zero(param_size);
	
	tMatrix rot = getRotationForFrame(joint, frame);
	tMatrix trans = getTranslationForFrame(joint, frame);
	tQuaternion quat = cMathUtil::RotMatToQuaternion(rot);
	tVector offset = tVector(trans(0, 3), trans(1, 3), trans(2, 3), 0);

	out_pose.segment(0, cKinTree::gPosDim) = offset.segment(0, cKinTree::gPosDim);
	out_pose.segment(cKinTree::gPosDim, cKinTree::gRotDim) = cMathUtil::QuatToVec(quat);
}

void cBVHReader::BuildJointPoseRevolute(int j, int frame, Eigen::VectorXd& out_pose) const
{
	const tJointData& joint = GetJointData(j);
	int param_size = cKinTree::GetJointParamSize(joint.mJointType);
	out_pose = Eigen::VectorXd::Zero(param_size);
	
	double val = motionData.data(frame, joint.channel_start);
	out_pose[0] = val;
}

void cBVHReader::BuildJointPosePlanar(int j, int frame, Eigen::VectorXd& out_pose) const
{
	const tJointData& joint = GetJointData(j);
	int param_size = cKinTree::GetJointParamSize(joint.mJointType);
	out_pose = Eigen::VectorXd::Zero(param_size);
	
	tMatrix trans = getTranslationForFrame(joint, frame);
	tVector offset = tVector(trans(0, 3), trans(1, 3), trans(2, 3), 0);

	out_pose[0] = offset[0];
	out_pose[1] = offset[1];
}

void cBVHReader::BuildJointPosePrismatic(int j, int frame, Eigen::VectorXd& out_pose) const
{
	const tJointData& joint = GetJointData(j);
	int param_size = cKinTree::GetJointParamSize(joint.mJointType);
	out_pose = Eigen::VectorXd::Zero(param_size);
	
	double val = motionData.data(frame, joint.channel_start);
	out_pose[0] = val;
}

void cBVHReader::BuildJointPoseFixed(int j, int frame, Eigen::VectorXd& out_pose) const
{
	const tJointData& joint = GetJointData(j);
	int param_size = cKinTree::GetJointParamSize(joint.mJointType);
	out_pose = Eigen::VectorXd::Zero(param_size);
}

void cBVHReader::BuildJointPoseSpherical(int j, int frame, Eigen::VectorXd& out_pose) const
{
	const tJointData& joint = GetJointData(j);
	int param_size = cKinTree::GetJointParamSize(joint.mJointType);
	out_pose = Eigen::VectorXd::Zero(param_size);
	
	tMatrix rot = getRotationForFrame(joint, frame);
	tQuaternion quat = cMathUtil::RotMatToQuaternion(rot);
	out_pose.segment(0, cKinTree::gRotDim) = cMathUtil::QuatToVec(quat);
}