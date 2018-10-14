#include "Trajectory.h"

cTrajectory::cTrajectory()
{
	mWrapMode = eWrapModeClamp;
	mAnchors.resize(0, 0);
}

cTrajectory::~cTrajectory()
{
}

void cTrajectory::Init(const Eigen::MatrixXd& anchors, eWrapMode mode)
{
	Clear();
	mWrapMode = mode;
	mAnchors = anchors;
}

void cTrajectory::Clear()
{
	mAnchors.resize(0, 0);
}

Eigen::VectorXd cTrajectory::Eval(double t)
{
	const int num_anchors = GetNumAnchors();

	double a_idx = PhaseToAnchor(t);
	int idx0 = static_cast<int>(a_idx);
	int idx1 = std::min(idx0 + 1, num_anchors - 1);
	double lerp = a_idx - idx0;

	Eigen::VectorXd x0 = GetAnchor(idx0);
	Eigen::VectorXd x1 = GetAnchor(idx1);
	Eigen::VectorXd output = (1 - lerp) * x0 + lerp * x1;
	return output;
}

int cTrajectory::GetNumAnchors() const
{
	return static_cast<int>(mAnchors.cols());
}

int cTrajectory::GetDim() const
{
	return static_cast<int>(mAnchors.rows());
}

int cTrajectory::GetSize() const
{
	return static_cast<int>(mAnchors.size());
}

Eigen::VectorXd cTrajectory::GetAnchor(int idx) const
{
	return mAnchors.col(idx);
}

void cTrajectory::SetAnchor(const Eigen::VectorXd& anchor, int idx)
{
	mAnchors.col(idx) = anchor;
}

double cTrajectory::NormalizePhase(double t) const
{
	double norm_t = 0;
	if (mWrapMode == eWrapModeClamp)
	{
		norm_t = cMathUtil::Clamp(t, 0.0, 1.0);
	}
	else if (mWrapMode == eWrapModeLoop)
	{
		norm_t = t - static_cast<int>(t);
		if (norm_t < 0)
		{
			norm_t = 1 + norm_t;
		}
		if (norm_t == 1)
		{
			norm_t = 0;
		}
	}
	else
	{
		assert(false); // unsupported wrap mode
	}

	return norm_t;
}

double cTrajectory::GetPhaseStep() const
{
	int num_anchors = GetNumAnchors();
	double phase_step = 1.0 / (num_anchors - 1);
	return phase_step;
}

double cTrajectory::PhaseToAnchor(double t) const
{
	double norm_t = NormalizePhase(t);
	int num_anchors = GetNumAnchors();

	if (mWrapMode == eWrapModeLoop)
	{
		if (norm_t == 1)
		{
			norm_t = 0;
		}
	}

	double idx = norm_t * (num_anchors - 1);
	return idx;
}

double cTrajectory::AnchorToPhase(int idx) const
{
	int num_anchors = GetNumAnchors();
	double phase = static_cast<double>(idx) / (num_anchors - 1);
	return phase;
}

void cTrajectory::Filter(double r)
{
	double phase_step = GetPhaseStep();
	int half_samples = static_cast<int>(2 * r / phase_step);
	int num_anchors = GetNumAnchors();

	Eigen::MatrixXd anchors = mAnchors;

	for (int i = 0; i < num_anchors; ++i)
	{
		double sum_weight = 0;
		double phase = AnchorToPhase(i);
		Eigen::VectorXd val = Eigen::VectorXd::Zero(GetDim());
		
		for (int s = -half_samples; s <= half_samples; ++s)
		{
			double d_phase = s * phase_step;
			double curr_phase = phase + d_phase;
			Eigen::VectorXd curr_val = Eval(curr_phase);
			double w = std::exp(-d_phase * d_phase / (r * r));

			sum_weight += w;
			val += w * curr_val;
		}
		val /= sum_weight;
		anchors.col(i) = val;
	}

	mAnchors = anchors;
}

void cTrajectory::Compress(double ratio)
{
	Filter(ratio / 2);
	int num_anchors = GetNumAnchors();
	int num_compressed_anchors = static_cast<int>(std::ceil(num_anchors * ratio));
	Eigen::MatrixXd new_anchors = Eigen::MatrixXd(GetDim(), num_compressed_anchors);
	for (int i = 0; i < num_compressed_anchors; ++i)
	{
		double curr_phase = i / (num_compressed_anchors - 1.0);
		Eigen::VectorXd curr_val = Eval(curr_phase);
		new_anchors.col(i) = curr_val;
	}
	mAnchors = new_anchors;
}

Eigen::VectorXd cTrajectory::Unroll() const
{
	Eigen::VectorXd unrolled(mAnchors.size());
	int dim = GetDim();
	for (int i = 0; i < mAnchors.cols(); ++i)
	{
		unrolled.segment(i * dim, dim) = mAnchors.col(i);
	}
	//Eigen::VectorXd unrolled = Eigen::VectorXd::Map(mAnchors.data(), mAnchors.size());
	return unrolled;
}

void cTrajectory::SetUnrolled(const Eigen::VectorXd& unrolled_data, int dim)
{
	assert(unrolled_data.size() % dim == 0);
	int num_anchors = static_cast<int>(unrolled_data.size()) / dim;
	mAnchors.resize(dim, num_anchors);
	for (int i = 0; i < num_anchors; ++i)
	{
		mAnchors.col(i) = unrolled_data.segment(i * dim, dim);
	}
}