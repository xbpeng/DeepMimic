#include "DrawPerturb.h"
#include "render/DrawUtil.h"
#include "sim/SimObj.h"

const double cDrawPerturb::gForceScale = 0.005;
const double cDrawPerturb::gTorqueScale = 0.00075;// * 0.25;

void cDrawPerturb::DrawForce(const tVector& pos, const tVector& force)
{
	const double len_scale = gForceScale;
	const double arrow_size = 0.1;
	tVector pos1 = pos + force * len_scale;

	cDrawUtil::SetColor(tVector(1, 0, 0, 0.5));
	cDrawUtil::DrawArrow3D(pos, pos1, arrow_size);
}

void cDrawPerturb::DrawTorque(const tVector& pos, const tVector& torque)
{
	const double torque_scale = gTorqueScale;
	const tVector color0 = tVector(1, 0, 0, 0.25);
	const tVector color1 = tVector(0, 1, 1, 0.25);

	tVector col;
	if (torque[2] < 0)
	{
		col = color0;
	}
	else
	{
		col = color1;
	}

	double mag = torque.norm();
	double r = mag * torque_scale;

	cDrawUtil::SetColor(tVector(col[0], col[1], col[2], col[3]));

	cDrawUtil::PushMatrixView();
	cDrawUtil::Translate(pos);
	cDrawUtil::DrawDisk(r);
	cDrawUtil::PopMatrixView();
}

void cDrawPerturb::Draw(const tPerturb& perturb)
{
	tPerturb::ePerturb type = perturb.mType;
	switch (type)
	{
	case tPerturb::ePerturbForce:
		DrawForce(perturb);
		break;
	case tPerturb::ePerturbTorque:
		DrawTorque(perturb);
		break;
	default:
		break;
	}
}

void cDrawPerturb::DrawForce(const tPerturb& perturb)
{
	tVector pos = perturb.mObj->LocalToWorldPos(perturb.mLocalPos);
	const tVector& force = perturb.mPerturb;
	DrawForce(pos, force);
}

void cDrawPerturb::DrawTorque(const tPerturb& perturb)
{
	tVector pos = perturb.mObj->LocalToWorldPos(perturb.mLocalPos);
	const tVector& torque = perturb.mPerturb;
	DrawTorque(pos, torque);
}