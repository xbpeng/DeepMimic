#pragma once

#include "scenes/SceneImitate.h"
#include "util/CircularBuffer.h"
#include "util/DynamicTimeWarper.h"

class cSceneImitateAMP : virtual public cSceneImitate
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static double TimeWarpCost(const Eigen::VectorXd* data0, const Eigen::VectorXd* data1);

	cSceneImitateAMP();
	virtual ~cSceneImitateAMP();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Init();
	virtual void Reset();

	virtual double CalcReward(int agent_id) const;
	virtual eTerminate CheckTerminate(int agent_id) const;

	virtual bool EnableAMPTaskReward() const;
	virtual int GetAMPObsSize() const;
	virtual void GetAMPObsOffset(Eigen::VectorXd& out_data) const;
	virtual void GetAMPObsScale(Eigen::VectorXd& out_data) const;
	virtual void GetAMPObsNormGroup(Eigen::VectorXi& out_data) const;
	virtual void RecordAMPObsAgent(int agent_id, Eigen::VectorXd& out_data);
	virtual void RecordAMPObsExpert(int agent_id, Eigen::VectorXd& out_data);
	
	virtual std::string GetName() const;

protected:
	bool mEnableAMPObsLocalRoot;

	Eigen::VectorXd mPrevPose;
	Eigen::VectorXd mPrevVel;

	bool mTestTimeWarp;
	std::unique_ptr<cDynamicTimeWarper> mTimeWarper;

	virtual void NewActionUpdate(int agent_id);
	virtual void InitHist();
	virtual void UpdateHist();

	virtual double CalcRewardTimeWarp(int agent_id) const;

	virtual int GetAMPObsPoseSize() const;
	virtual int GetAMPObsVelSize() const;
	virtual const cMotion* SampleExpertMotion(const cKinCharacter* kin_char) const;

	virtual void BuildAMPObs(int agent_id, const Eigen::VectorXd& prev_pose, const Eigen::VectorXd& prev_vel,
							const Eigen::VectorXd& pose, const Eigen::VectorXd& vel,
							double ground_h, Eigen::VectorXd& out_data) const;
	virtual int RecordAMPObsPose(int agent_id, const Eigen::VectorXd& pose, double ground_h, const tQuaternion& ref_origin_rot,
								int param_offset, Eigen::VectorXd& out_data) const;
	virtual int RecordAMPObsVel(int agent_id, const Eigen::VectorXd& vel, const tQuaternion& ref_origin_rot, int param_offset,
								Eigen::VectorXd& out_data) const;

	virtual bool EnableTestTimeWarp() const;
	virtual int GetTimeWarpDataDim() const;
	virtual void BuildTimeWarper();
	virtual void ResetTimeWarper();
	virtual void UpdateTimeWarper();
	virtual void BuildTimeWarpData(const cCharacter& character, Eigen::VectorXd& out_data) const;
};