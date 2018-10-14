#pragma once

#include <memory>

#include "sim/Perturb.h"

class cPerturbManager
{
public:
	cPerturbManager();
	virtual ~cPerturbManager();

	virtual void Update(double time_step);
	virtual void Clear();
	virtual void AddPerturb(const tPerturb& perturb);

	virtual int GetNumPerturbs() const;
	virtual const tPerturb& GetPerturb(int i) const;

protected:
	tEigenArr<tPerturb> mPerturbs;

	virtual void UpdatePerturbs(double time_step);
};