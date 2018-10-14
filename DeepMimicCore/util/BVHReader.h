/*
 * BVHReader.h
 *
 *  Created on: Nov 4, 2016
 *      Author: Glen
 */

#ifndef UTIL_BVHREADER_H_
#define UTIL_BVHREADER_H_

#include <string>
#include <vector>
#include <set>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <memory>
#include <cctype>
#include "MathUtil.h"
#include "anim/KinTree.h"
#include "anim/Motion.h"

class cBVHReader {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const std::string gReserved[];
	static const std::string gChannelNames[];

	enum Channels
	{
		Xposition = 0,
		Yposition,
		Zposition,
		Xrotation,
		Yrotation,
		Zrotation
	};

	struct tJointData
	{
		// joint name
		std::string name;

		cKinTree::eJointType mJointType;

		// joint parent
		int parent;

		// offset data
		tVector offset;

		// num of channels joint has
		size_t num_channels = 0;

		// ordered list of channels
		std::vector<Channels>channels_order;

		// joint's children
		std::vector<int> children;

		// local transofrmation matrix (premultiplied with parents'
		tMatrix matrix;

		// index of joint's channel data in motion array
		size_t channel_start = 0;

		int CalcPoseDim() const;
		bool IsRoot() const;
		bool IsValid() const;
	};

	struct PoseData
	{
		size_t num_frames;            // 
		size_t num_motion_channels = 0;   // 
		double frame_step = 0.0;     // The timestep of the recording
									 // ordered list of channels
		std::vector<Channels> channels_types; // Used to find rotations and convert them to radians
		Eigen::MatrixXd data;                        // 

		void Clear();
	};

	cBVHReader();
	virtual ~cBVHReader();

	virtual void Clear();

	virtual int parseJoint(std::istream& stream, int joint_id = gInvalidIdx);
	virtual void parseHierarchy(std::istream& stream);
	virtual void parseMotion(std::istream& stream);

	virtual void printJointHeirachy(const tJointData& joint) const;

	virtual void printData() const;

	virtual void parseBVH(const std::string& filename);

	virtual size_t GetNumFrames() const { return motionData.num_frames; }
	virtual Eigen::MatrixXd getMotoinData() const { return this->motionData.data; }
	virtual void SetModelTransform(const tMatrix& trans);

	// virtual void processMotionClips();
	/// Should return a cMotion, for now really only supports two step locations.
	
	virtual bool GetJointLocation(std::string jointName, size_t frame_, tVector & position,
									tMatrix & transform) const;
	virtual bool GetJointLocation(const tJointData& joint, std::string jointName, size_t frame_, tVector & position,
									tMatrix & transform) const;

	virtual tMatrix getTransformationForFrame(const tJointData& joint, size_t frame) const;
	virtual tMatrix getTranslationForFrame(const tJointData& joint, size_t frame) const;
	virtual tMatrix getRotationForFrame(const tJointData& joint, size_t frame) const;

	virtual const tJointData& GetRootJoint() const;
	virtual const tJointData& GetJointData(int joint_id) const;
	virtual int FindJoint(std::string joint_name) const;

	virtual int GetNumJoints() const;
	virtual void BuildJointMat(Eigen::MatrixXd& out_joint_mat) const;
	virtual void BuildMotion(double target_framerate, cMotion& out_motion) const;
	virtual int CalcPoseDim() const;

	virtual double GetFramerate() const;

private:
	// Stores the representation of the joint heirachy
	std::vector<tJointData> mJointData;
	// Stores the pose data for each frame;
	PoseData motionData;
	tMatrix mModelTrans;
	size_t mChannelCounter;

	virtual tMatrix getTransformationForFrame(size_t channel_start, const std::vector<Channels> & channels_order, const Eigen::MatrixXd & frame_data, size_t frame) const;
	virtual tMatrix getTranslationForFrame(size_t channel_start, const std::vector<Channels> & channels_order, const Eigen::MatrixXd & frame_data, size_t frame) const;
	virtual tMatrix getRotationForFrame(size_t channel_start, const std::vector<Channels> & channels_order, const Eigen::MatrixXd & frame_data, size_t frame) const;

	virtual tJointData& GetJointData(int joint_id);
	virtual int AddJoint();
	virtual void FetchValidJoints(std::vector<int>& out_joints) const;
	virtual cKinTree::eJointType CalcJointType(const tJointData& joint_data) const;
	virtual void ConvertFrameToPose(int f, Eigen::VectorXd& out_pose) const;

	virtual void BuildJointPose(int j, int frame, Eigen::VectorXd& out_pose) const;
	virtual void BuildJointPoseRoot(int j, int frame, Eigen::VectorXd& out_pose) const;
	virtual void BuildJointPoseRevolute(int j, int frame, Eigen::VectorXd& out_pose) const;
	virtual void BuildJointPosePlanar(int j, int frame, Eigen::VectorXd& out_pose) const;
	virtual void BuildJointPosePrismatic(int j, int frame, Eigen::VectorXd& out_pose) const;
	virtual void BuildJointPoseFixed(int j, int frame, Eigen::VectorXd& out_pose) const;
	virtual void BuildJointPoseSpherical(int j, int frame, Eigen::VectorXd& out_pose) const;
};

// trim from start
static inline std::string &ltrim(std::string &s) {
	s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
	return s;
}

// trim from end
static inline std::string &rtrim(std::string &s) {
	s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
	return s;
}

// trim from both ends
static inline std::string &trim(std::string &s) {
	return ltrim(rtrim(s));
}

#endif /* UTIL_BVHREADER_H_ */
