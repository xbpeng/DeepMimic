#include "DrawWorld.h"
#include "render/DrawPerturb.h"

void cDrawWorld::DrawPerturbs(const cWorld& world)
{
	const cPerturbManager& perturb_man = world.GetPerturbManager();
	int num_perturbs = perturb_man.GetNumPerturbs();
	for (int p = 0; p < num_perturbs; ++p)
	{
		const tPerturb& perturb = perturb_man.GetPerturb(p);
		cDrawPerturb::Draw(perturb);
	}
}