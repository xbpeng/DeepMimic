#pragma once

#include <vector>
#include <string>
#include <memory>
#include <functional>

#include "util/MathUtil.h"
#include "util/ArgParser.h"
#include "util/Timer.h"

class cScene
{
public:
	virtual ~cScene();
	
	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Init();
	virtual void Clear();
	virtual void Reset();
	virtual void Update(double timestep);

	virtual void Draw();
	virtual void Keyboard(unsigned char key, double device_x, double device_y);
	virtual void MouseClick(int button, int state, double device_x, double device_y);
	virtual void MouseMove(double device_x, double device_y);
	virtual void Reshape(int w, int h);

	virtual void Shutdown();
	virtual bool IsDone() const;
	virtual double GetTime() const;

	virtual bool HasRandSeed() const;
	virtual void SetRandSeed(unsigned long seed);
	virtual unsigned long GetRandSeed() const;

	virtual bool IsEpisodeEnd() const;
	virtual bool CheckValidEpisode() const;

	virtual std::string GetName() const = 0;

protected:

	cRand mRand;
	bool mHasRandSeed;
	unsigned long mRandSeed;

	std::shared_ptr<cArgParser> mArgParser;
	cTimer::tParams mTimerParams;
	cTimer mTimer;

	cScene();

	virtual void ResetParams();
	virtual void ResetScene();

	virtual void InitTimers();
	virtual void ResetTimers();
	virtual void UpdateTimers(double timestep);
};