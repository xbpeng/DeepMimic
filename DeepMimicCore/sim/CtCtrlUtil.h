#pragma once

#include "util/MathUtil.h"

class cCtCtrlUtil
{
public:
	static const double gMaxPDExpVal;

	static int GetParamDimTorque(const Eigen::MatrixXd& joint_mat, int joint_id);
	static int GetParamDimVel(const Eigen::MatrixXd& joint_mat, int joint_id);
	static int GetParamDimPD(const Eigen::MatrixXd& joint_mat, int joint_id);
	
	static void BuildBoundsTorque(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max);
	static void BuildBoundsVel(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max);
	static void BuildBoundsPD(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max);

	static void BuildOffsetScaleTorque(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale);
	static void BuildOffsetScaleVel(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale);
	static void BuildOffsetScalePD(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale);
	
protected:
	static void BuildBoundsPDRevolute(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max);
	static void BuildBoundsPDPrismatic(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max);
	static void BuildBoundsPDPlanar(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max);
	static void BuildBoundsPDFixed(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max);
	static void BuildBoundsPDSpherical(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max);

	static void BuildOffsetScalePDRevolute(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale);
	static void BuildOffsetScalePDPrismatic(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale);
	static void BuildOffsetScalePDPlanar(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale);
	static void BuildOffsetScalePDFixed(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale);
	static void BuildOffsetScalePDSpherical(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale);
};