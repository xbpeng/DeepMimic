#include "GraphUtil.h"

cGraphUtil::tBarPlot::tBarPlot()
{
	mMinVal = 0;
	mMaxVal = 1;
	mBaseVal = 0;
	mVals.resize(0);
	mColors.clear();
}

void cGraphUtil::DrawBarPlot(const tBarPlot& bar_plot, const tVector& pos, const tVector& size)
{
	cDrawUtil::SetLineWidth(1);
	cDrawUtil::SetColor(tVector(1, 1, 1, 0.5));
	cDrawUtil::DrawRect(pos, size);

	const Eigen::VectorXd& vals = bar_plot.mVals;
	double min_val = bar_plot.mMinVal;
	double max_val = bar_plot.mMaxVal; 
	double base_val = bar_plot.mBaseVal;
	const tVectorArr& cols = bar_plot.mColors;

	int num_vals = static_cast<int>(vals.size());
	int num_cols = static_cast<int>(cols.size());

	double norm_base_val = (base_val - min_val) / (max_val - min_val);

	for (int i = 0; i < num_vals; ++i)
	{
		double curr_val = vals[i];
		double norm_curr_val = (curr_val - min_val) / (max_val - min_val);

		tVector bar_min = tVector::Zero();
		tVector bar_max = tVector::Zero();
		
		bar_min[0] = static_cast<double>(i) / num_vals;
		bar_max[0] = static_cast<double>(i + 1) / num_vals;

		if (curr_val < base_val)
		{
			bar_min[1] = norm_curr_val;
			bar_max[1] = norm_base_val;
		}
		else
		{
			bar_min[1] = norm_base_val;
			bar_max[1] = norm_curr_val;
		}

		bar_min[1] = cMathUtil::Clamp(bar_min[1], 0.0, 1.0);
		bar_max[1] = cMathUtil::Clamp(bar_max[1], 0.0, 1.0);
		bar_min = size.cwiseProduct(bar_min) + pos - 0.5 * size;
		bar_max = size.cwiseProduct(bar_max) + pos - 0.5 * size;

		const tVector& col = cols[i % num_cols];
		cDrawUtil::SetColor(col);
		cDrawUtil::DrawRect(0.5 * (bar_max + bar_min), bar_max - bar_min);
	}

	double base_y = size[1] * norm_base_val + pos[1] - 0.5 * size[1];
	tVector base0 = pos;
	tVector base1 = pos;
	base0[0] += -0.5 * size[0];
	base1[0] += 0.5 * size[0];
	base0[1] = base_y;
	base1[1] = base_y;

	cDrawUtil::SetColor(tVector(0, 0, 0, 1));
	cDrawUtil::DrawLine(base0, base1);
	cDrawUtil::DrawRect(pos, size, cDrawUtil::eDrawWireSimple);
}