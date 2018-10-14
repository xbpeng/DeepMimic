#pragma once

class cAnnealer
{
public:
	enum eType
	{
		eTypeLinear,
		eTypePow,
		eTypeMax
	};

	struct tParams
	{
		eType mType;
		double mPow;
		tParams();
	};

	cAnnealer();
	virtual ~cAnnealer();

	virtual void Init(const tParams& params);
	virtual double Eval(double t);

protected:
	tParams mParams;
};