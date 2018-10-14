#include "PerturbManager.h"

cPerturbManager::cPerturbManager()
{
}

cPerturbManager::~cPerturbManager()
{
}

void cPerturbManager::Update(double time_step)
{
	UpdatePerturbs(time_step);
}

void cPerturbManager::Clear()
{
	mPerturbs.clear();
}

void cPerturbManager::AddPerturb(const tPerturb& perturb)
{
	if (perturb.IsValid())
	{
		mPerturbs.push_back(perturb);
	}
}

int cPerturbManager::GetNumPerturbs() const
{
	return static_cast<int>(mPerturbs.size());
}

const tPerturb& cPerturbManager::GetPerturb(int i) const
{
	return mPerturbs[i];
}

void cPerturbManager::UpdatePerturbs(double time_step)
{
	int num_perturbs = GetNumPerturbs();
	int idx = 0;
	for (int i = 0; i < num_perturbs; ++i)
	{
		tPerturb& curr_perturb = mPerturbs[i];
		if (!curr_perturb.HasExpired())
		{
			curr_perturb.Update(time_step);
			mPerturbs[idx] = curr_perturb;
			++idx;
		}
	}
	mPerturbs.resize(idx);
}