#include "Annealer.h"
#include "util/MathUtil.h"

cAnnealer::tParams::tParams()
{
	mType = eTypeLinear;
	mPow = 1;
}

cAnnealer::cAnnealer()
{
}

cAnnealer::~cAnnealer()
{
}

void cAnnealer::Init(const tParams& params)
{
	mParams = params;
}

double cAnnealer::Eval(double t)
{
	double val = 0;
	t = cMathUtil::Clamp(t, 0.0, 1.0);

	switch (mParams.mType)
	{
	case eTypeLinear:
		val = t;
		break;
	case eTypePow:
		val = std::pow(t, mParams.mPow);
		break;
	default:
		assert(false); // unsupported
		break;
	}
	return val;
}