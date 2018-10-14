#include "RLScene.h"

cRLScene::cRLScene()
{
	mMode = eModeTrain;
	mSampleCount = 0;
}

cRLScene::~cRLScene()
{
}

void cRLScene::Init()
{
	cScene::Init();
	mSampleCount = 0;
}

void cRLScene::Clear()
{
	cScene::Clear();
	mSampleCount = 0;
}


double cRLScene::GetRewardFail(int agent_id)
{
	return GetRewardMin(agent_id);
}

double cRLScene::GetRewardSucc(int agent_id)
{
	return GetRewardMax(agent_id);
}

bool cRLScene::IsEpisodeEnd() const
{
	bool is_end = cScene::IsEpisodeEnd();
	eTerminate termin = eTerminateNull;
	for (int i = 0; i < GetNumAgents(); ++i)
	{
		termin = CheckTerminate(i);
		if (termin != eTerminateNull)
		{
			is_end = true;
			break;
		}
	}
	return is_end;
}

cRLScene::eTerminate cRLScene::CheckTerminate(int agent_id) const
{
	return eTerminateNull;
}

void cRLScene::SetSampleCount(int count)
{
	mSampleCount = count;
}

void cRLScene::SetMode(eMode mode)
{
	mMode = mode;
}