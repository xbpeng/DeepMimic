#pragma once

#include <string>

class cTimer
{
public:
	enum eType
	{
		eTypeUniform,
		eTypeExp,
		eTypeMax
	};

	struct tParams
	{
		eType mType;
		double mTimeMin;
		double mTimeMax;
		double mTimeExp;

		tParams();
		tParams Blend(const tParams& other, double lerp);
	};

	static eType ParseTypeStr(const std::string& str);

	cTimer();
	virtual ~cTimer();

	virtual void Init(const tParams& params);
	virtual void Reset();
	virtual void Update(double timestep);
	virtual bool IsEnd() const;

	virtual double GetTime() const;
	virtual double GetMaxTime() const;
	virtual void SetTime(double time);
	virtual void SetMaxTime(double time);

	virtual const tParams& GetParams() const;
	virtual void SetParams(const tParams& params);

protected:

	tParams mParams;
	double mMaxTime;
	double mTime;
};