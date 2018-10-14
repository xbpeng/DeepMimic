#pragma once

#include "render/DrawUtil.h"

class cGraphUtil
{
public:
	struct tBarPlot
	{
		double mMinVal;
		double mMaxVal;
		double mBaseVal;
		Eigen::VectorXd mVals;
		tVectorArr mColors;

		tBarPlot();
	};

	static void DrawBarPlot(const tBarPlot& bar_plot, const tVector& pos, const tVector& size);
};