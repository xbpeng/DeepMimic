#include "CtCtrlUtil.h"
#include <limits>
#include "anim/KinTree.h"

const double gDefaultOffsetPDBound = 10;
const double gDefaultRotatePDBound = M_PI;
const double cCtCtrlUtil::gMaxPDExpVal = 2.0 * M_PI;


int cCtCtrlUtil::GetParamDimTorque(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	int dim = 0;
	cKinTree::eJointType joint_type = cKinTree::GetJointType(joint_mat, joint_id);
	if (joint_type == cKinTree::eJointTypeSpherical)
	{
		dim = 3;
	}
	else
	{
		dim = cKinTree::GetParamSize(joint_mat, joint_id);
	}
	return dim;
}

int cCtCtrlUtil::GetParamDimVel(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	int dim = GetParamDimTorque(joint_mat, joint_id);
	return dim;
}

int cCtCtrlUtil::GetParamDimPD(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	int dim = GetParamDimTorque(joint_mat, joint_id);
	return dim;
}

void cCtCtrlUtil::BuildBoundsTorque(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max)
{
	int joint_dim = cKinTree::GetParamSize(joint_mat, joint_id);
	cKinTree::eJointType joint_type = cKinTree::GetJointType(joint_mat, joint_id);
	out_min = std::numeric_limits<double>::quiet_NaN() * Eigen::VectorXd::Ones(joint_dim);
	out_max = std::numeric_limits<double>::quiet_NaN() * Eigen::VectorXd::Ones(joint_dim);

	double torque_lim = cKinTree::GetTorqueLimit(joint_mat, joint_id);
	double force_lim = cKinTree::GetForceLimit(joint_mat, joint_id);

	switch (joint_type)
	{
	case cKinTree::eJointTypeRevolute:
		out_min.fill(-torque_lim);
		out_max.fill(torque_lim);
		break;
	case cKinTree::eJointTypePrismatic:
		out_min.fill(-force_lim);
		out_max.fill(force_lim);
		break;
	case cKinTree::eJointTypePlanar:
		out_min.fill(-force_lim);
		out_max.fill(force_lim);
		out_min[joint_dim - 1] = -torque_lim;
		out_max[joint_dim - 1] = torque_lim;
		break;
	case cKinTree::eJointTypeFixed:
		break;
	case cKinTree::eJointTypeSpherical:
		out_min.segment(0, joint_dim - 1).fill(-torque_lim);
		out_max.segment(0, joint_dim - 1).fill(torque_lim);
		out_min[joint_dim - 1] = 0;
		out_max[joint_dim - 1] = 0;
		break;
	default:
		assert(false); // unsupported joint type
		break;
	}
}

void cCtCtrlUtil::BuildBoundsVel(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max)
{
	const double max_ang_vel = 20 * M_PI;
	const double max_lin_vel = 5;

	int joint_dim = cKinTree::GetParamSize(joint_mat, joint_id);
	cKinTree::eJointType joint_type = cKinTree::GetJointType(joint_mat, joint_id);
	
	out_min = std::numeric_limits<double>::quiet_NaN() * Eigen::VectorXd::Ones(joint_dim);
	out_max = std::numeric_limits<double>::quiet_NaN() * Eigen::VectorXd::Ones(joint_dim);

	switch (joint_type)
	{
	case cKinTree::eJointTypeRevolute:
		out_min.fill(-max_ang_vel);
		out_max.fill(max_ang_vel);
		break;
	case cKinTree::eJointTypePrismatic:
		out_min.fill(-max_lin_vel);
		out_max.fill(max_lin_vel);
		break;
	case cKinTree::eJointTypePlanar:
		out_min.fill(-max_lin_vel);
		out_max.fill(max_lin_vel);
		out_min[joint_dim - 1] = -max_ang_vel;
		out_max[joint_dim - 1] = max_ang_vel;
		break;
	case cKinTree::eJointTypeFixed:
		break;
	case cKinTree::eJointTypeSpherical:
		out_min.segment(0, joint_dim - 1).fill(-max_ang_vel);
		out_max.segment(0, joint_dim - 1).fill(max_ang_vel);
		out_min[joint_dim - 1] = 0;
		out_max[joint_dim - 1] = 0;
		break;
	default:
		assert(false); // unsupported joint type
		break;
	}
}

void cCtCtrlUtil::BuildBoundsPD(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max)
{
	cKinTree::eJointType joint_type = cKinTree::GetJointType(joint_mat, joint_id);
	switch (joint_type)
	{
	case cKinTree::eJointTypeRevolute:
		BuildBoundsPDRevolute(joint_mat, joint_id, out_min, out_max);
		break;
	case cKinTree::eJointTypePrismatic:
		BuildBoundsPDPrismatic(joint_mat, joint_id, out_min, out_max);
		break;
	case cKinTree::eJointTypePlanar:
		BuildBoundsPDPlanar(joint_mat, joint_id, out_min, out_max);
		break;
	case cKinTree::eJointTypeFixed:
		BuildBoundsPDFixed(joint_mat, joint_id, out_min, out_max);
		break;
	case cKinTree::eJointTypeSpherical:
		BuildBoundsPDSpherical(joint_mat, joint_id, out_min, out_max);
		break;
	default:
		assert(false); // unsupported joint type
		break;
	}
}


void cCtCtrlUtil::BuildOffsetScaleTorque(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale)
{
	const double default_torque_lim = 300;
	const double default_force_lim = 3000;

	int joint_dim = cKinTree::GetParamSize(joint_mat, joint_id);
	cKinTree::eJointType joint_type = cKinTree::GetJointType(joint_mat, joint_id);
	out_offset = Eigen::VectorXd::Zero(joint_dim);
	out_scale = Eigen::VectorXd::Ones(joint_dim);

	double torque_lim = cKinTree::GetTorqueLimit(joint_mat, joint_id);
	double force_lim = cKinTree::GetForceLimit(joint_mat, joint_id);

	if (!std::isfinite(torque_lim))
	{
		torque_lim = default_torque_lim;
	}

	if (!std::isfinite(force_lim))
	{
		force_lim = default_force_lim;
	}

	switch (joint_type)
	{
	case cKinTree::eJointTypeRevolute:
		out_scale.fill(1 / torque_lim);
		break;
	case cKinTree::eJointTypePrismatic:
		out_scale.fill(1 / force_lim);
		break;
	case cKinTree::eJointTypePlanar:
		out_scale.fill(1 / force_lim);
		out_scale[joint_dim - 1] = 1 / torque_lim;
		break;
	case cKinTree::eJointTypeFixed:
		break;
	case cKinTree::eJointTypeSpherical:
		out_scale.fill(1 / torque_lim);
		out_scale[joint_dim - 1] = 0;
		break;
	default:
		assert(false); // unsupported joint type
		break;
	}
}

void cCtCtrlUtil::BuildOffsetScaleVel(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale)
{
	int joint_dim = cKinTree::GetParamSize(joint_mat, joint_id);
	cKinTree::eJointType joint_type = cKinTree::GetJointType(joint_mat, joint_id);

	const double ang_vel_scale = 1 / (10 * M_PI);
	const double lin_vel_scale = 1 / 2.5;

	out_offset = Eigen::VectorXd::Zero(joint_dim);
	out_scale = Eigen::VectorXd::Ones(joint_dim);

	switch (joint_type)
	{
	case cKinTree::eJointTypeRevolute:
		out_scale.fill(ang_vel_scale);
		break;
	case cKinTree::eJointTypePrismatic:
		out_scale.fill(lin_vel_scale);
		break;
	case cKinTree::eJointTypePlanar:
		out_scale.fill(lin_vel_scale);
		out_scale[joint_dim - 1] = ang_vel_scale;
		break;
	case cKinTree::eJointTypeFixed:
		break;
	case cKinTree::eJointTypeSpherical:
		out_scale.segment(0, joint_dim - 1).fill(ang_vel_scale);
		out_scale[joint_dim - 1] = 1;
		break;
	default:
		assert(false); // unsupported joint type
		break;
	}
}

void cCtCtrlUtil::BuildOffsetScalePD(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale)
{
	cKinTree::eJointType joint_type = cKinTree::GetJointType(joint_mat, joint_id);
	switch (joint_type)
	{
	case cKinTree::eJointTypeRevolute:
		BuildOffsetScalePDRevolute(joint_mat, joint_id, out_offset, out_scale);
		break;
	case cKinTree::eJointTypePrismatic:
		BuildOffsetScalePDPrismatic(joint_mat, joint_id, out_offset, out_scale);
		break;
	case cKinTree::eJointTypePlanar:
		BuildOffsetScalePDPlanar(joint_mat, joint_id, out_offset, out_scale);
		break;
	case cKinTree::eJointTypeFixed:
		BuildOffsetScalePDFixed(joint_mat, joint_id, out_offset, out_scale);
		break;
	case cKinTree::eJointTypeSpherical:
		BuildOffsetScalePDSpherical(joint_mat, joint_id, out_offset, out_scale);
		break;
	default:
		assert(false); // unsupported joint type
		break;
	}
}



void cCtCtrlUtil::BuildBoundsPDRevolute(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max)
{
	cKinTree::eJointType joint_type = cKinTree::GetJointType(joint_mat, joint_id);
	assert(joint_type == cKinTree::eJointTypeRevolute);

	int joint_dim = cKinTree::GetParamSize(joint_mat, joint_id);
	out_min = Eigen::VectorXd::Zero(joint_dim);
	out_max = Eigen::VectorXd::Zero(joint_dim);

	tVector lim_low = cKinTree::GetJointLimLow(joint_mat, joint_id);
	tVector lim_high = cKinTree::GetJointLimHigh(joint_mat, joint_id);

	for (int i = 0; i < joint_dim; ++i)
	{
		double val_low = lim_low[i];
		double val_high = lim_high[i];
		bool valid_lim = val_high >= val_low;
		if (!valid_lim)
		{
			val_low = -gDefaultRotatePDBound;
			val_high = gDefaultRotatePDBound;
		}

		double mean_val = 0.5 * (val_high + val_low);
		double delta = val_high - val_low;
		val_low = mean_val - 2 * delta;
		val_high = mean_val + 2 * delta;

		out_min[i] = val_low;
		out_max[i] = val_high;
	}
}

void cCtCtrlUtil::BuildBoundsPDPrismatic(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max)
{
	cKinTree::eJointType joint_type = cKinTree::GetJointType(joint_mat, joint_id);
	assert(joint_type == cKinTree::eJointTypePrismatic);

	int joint_dim = cKinTree::GetParamSize(joint_mat, joint_id);
	out_min = Eigen::VectorXd::Zero(joint_dim);
	out_max = Eigen::VectorXd::Zero(joint_dim);

	tVector lim_low = cKinTree::GetJointLimLow(joint_mat, joint_id);
	tVector lim_high = cKinTree::GetJointLimHigh(joint_mat, joint_id);

	for (int i = 0; i < joint_dim; ++i)
	{
		double val_low = lim_low[i];
		double val_high = lim_high[i];
		bool valid_lim = val_high >= val_low;
		if (!valid_lim)
		{
			val_low = -gDefaultOffsetPDBound;
			val_high = gDefaultOffsetPDBound;
		}

		double mean_val = 0.5 * (val_high + val_low);
		double delta = val_high - val_low;
		val_low = mean_val - delta;
		val_high = mean_val + delta;
		out_min[i] = val_low;
		out_max[i] = val_high;
	}
}

void cCtCtrlUtil::BuildBoundsPDPlanar(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max)
{
	cKinTree::eJointType joint_type = cKinTree::GetJointType(joint_mat, joint_id);
	assert(joint_type == cKinTree::eJointTypePlanar);

	int joint_dim = cKinTree::GetParamSize(joint_mat, joint_id);
	out_min = Eigen::VectorXd::Zero(joint_dim);
	out_max = Eigen::VectorXd::Zero(joint_dim);

	tVector lim_low = cKinTree::GetJointLimLow(joint_mat, joint_id);
	tVector lim_high = cKinTree::GetJointLimHigh(joint_mat, joint_id);
	
	for (int i = 0; i < joint_dim; ++i)
	{
		double val_low = lim_low[i];
		double val_high = lim_high[i];
		bool valid_lim = val_high >= val_low;
		if (!valid_lim)
		{
			val_low = -gDefaultOffsetPDBound;
			val_high = gDefaultOffsetPDBound;
		}
		out_min[i] = val_low;
		out_max[i] = val_high;
	}
}

void cCtCtrlUtil::BuildBoundsPDFixed(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max)
{
	cKinTree::eJointType joint_type = cKinTree::GetJointType(joint_mat, joint_id);
	assert(joint_type == cKinTree::eJointTypeFixed);

	int joint_dim = cKinTree::GetParamSize(joint_mat, joint_id);
	out_min = Eigen::VectorXd::Zero(joint_dim);
	out_max = Eigen::VectorXd::Zero(joint_dim);
}

void cCtCtrlUtil::BuildBoundsPDSpherical(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max)
{
	cKinTree::eJointType joint_type = cKinTree::GetJointType(joint_mat, joint_id);
	assert(joint_type == cKinTree::eJointTypeSpherical);

	int param_dim = gExpMapDim;
	out_min = -gMaxPDExpVal * Eigen::VectorXd::Ones(param_dim);
	out_max = gMaxPDExpVal * Eigen::VectorXd::Ones(param_dim);
}


void cCtCtrlUtil::BuildOffsetScalePDRevolute(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale)
{
	cKinTree::eJointType joint_type = cKinTree::GetJointType(joint_mat, joint_id);
	assert(joint_type == cKinTree::eJointTypeRevolute);

	int joint_dim = cKinTree::GetParamSize(joint_mat, joint_id);
	out_offset = Eigen::VectorXd::Zero(joint_dim);
	out_scale = Eigen::VectorXd::Ones(joint_dim);

	tVector lim_low = cKinTree::GetJointLimLow(joint_mat, joint_id);
	tVector lim_high = cKinTree::GetJointLimHigh(joint_mat, joint_id);

	for (int i = 0; i < joint_dim; ++i)
	{
		double val_low = lim_low[i];
		double val_high = lim_high[i];
		bool valid_lim = val_high >= val_low;
		if (!valid_lim)
		{
			val_low = -gDefaultRotatePDBound;
			val_high = gDefaultRotatePDBound;
		}

		double curr_offset = -0.5 * (val_high + val_low);
		double curr_scale = 1 / (val_high - val_low);
		curr_scale *= 0.5;

		out_offset[i] = curr_offset;
		out_scale[i] = curr_scale;
	}
}

void cCtCtrlUtil::BuildOffsetScalePDPrismatic(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale)
{
	cKinTree::eJointType joint_type = cKinTree::GetJointType(joint_mat, joint_id);
	assert(joint_type == cKinTree::eJointTypePrismatic);

	int joint_dim = cKinTree::GetParamSize(joint_mat, joint_id);
	out_offset = Eigen::VectorXd::Zero(joint_dim);
	out_scale = Eigen::VectorXd::Ones(joint_dim);

	tVector lim_low = cKinTree::GetJointLimLow(joint_mat, joint_id);
	tVector lim_high = cKinTree::GetJointLimHigh(joint_mat, joint_id);

	for (int i = 0; i < joint_dim; ++i)
	{
		double val_low = lim_low[i];
		double val_high = lim_high[i];
		bool valid_lim = val_high >= val_low;
		if (!valid_lim)
		{
			val_low = -gDefaultOffsetPDBound;
			val_high = gDefaultOffsetPDBound;
		}
		out_offset[i] = -0.5 * (val_high + val_low);
		out_scale[i] = 2 / (val_high - val_low);
	}
}

void cCtCtrlUtil::BuildOffsetScalePDPlanar(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale)
{
	cKinTree::eJointType joint_type = cKinTree::GetJointType(joint_mat, joint_id);
	assert(joint_type == cKinTree::eJointTypePlanar);

	int joint_dim = cKinTree::GetParamSize(joint_mat, joint_id);
	out_offset = Eigen::VectorXd::Zero(joint_dim);
	out_scale = Eigen::VectorXd::Ones(joint_dim);

	tVector lim_low = cKinTree::GetJointLimLow(joint_mat, joint_id);
	tVector lim_high = cKinTree::GetJointLimHigh(joint_mat, joint_id);

	for (int i = 0; i < joint_dim; ++i)
	{
		double val_low = lim_low[i];
		double val_high = lim_high[i];
		bool valid_lim = val_high >= val_low;
		if (!valid_lim)
		{
			val_low = -gDefaultOffsetPDBound;
			val_high = gDefaultOffsetPDBound;
		}
		out_offset[i] = -0.5 * (val_high + val_low);
		out_scale[i] = 2 / (val_high - val_low);
	}
}

void cCtCtrlUtil::BuildOffsetScalePDFixed(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale)
{
	cKinTree::eJointType joint_type = cKinTree::GetJointType(joint_mat, joint_id);
	assert(joint_type == cKinTree::eJointTypeFixed);

	int joint_dim = cKinTree::GetParamSize(joint_mat, joint_id);
	out_offset = Eigen::VectorXd::Zero(joint_dim);
	out_scale = Eigen::VectorXd::Ones(joint_dim);
}

void cCtCtrlUtil::BuildOffsetScalePDSpherical(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale)
{
	const double scale = 2.0 / gMaxPDExpVal;

	cKinTree::eJointType joint_type = cKinTree::GetJointType(joint_mat, joint_id);
	assert(joint_type == cKinTree::eJointTypeSpherical);

	int param_dim = gExpMapDim;
	out_offset = Eigen::VectorXd::Zero(param_dim);
	out_scale = scale * Eigen::VectorXd::Ones(param_dim);
}