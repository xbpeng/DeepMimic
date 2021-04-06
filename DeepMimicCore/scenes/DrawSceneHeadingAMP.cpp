#include "DrawSceneHeadingAMP.h"
#include "SceneHeadingAMP.h"
#include "render/DrawUtil.h"

const double gTargetSpeedDelta = 0.2;

cDrawSceneHeadingAMP::cDrawSceneHeadingAMP()
{
}

cDrawSceneHeadingAMP::~cDrawSceneHeadingAMP()
{
}

void cDrawSceneHeadingAMP::Reset()
{
	auto scene = dynamic_cast<cSceneHeadingAMP*>(mScene.get());
	scene->EnableTargetPos(false);
	scene->EnableRandSpeed(true);

	cDrawSceneTargetAMP::Reset();
}

void cDrawSceneHeadingAMP::Keyboard(unsigned char key, double device_x, double device_y)
{
	cDrawSceneTargetAMP::Keyboard(key, device_x, device_y);
	KeyboardHeading(key, device_x, device_y);
}

void cDrawSceneHeadingAMP::BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cSceneHeadingAMP>(new cSceneHeadingAMP());
}

void cDrawSceneHeadingAMP::KeyboardHeading(unsigned char key, double device_x, double device_y)
{
	switch (key)
	{
	case '1':
		ChangeTargetSpeed(-gTargetSpeedDelta);
		break;
	case '2':
		ChangeTargetSpeed(gTargetSpeedDelta);
		break;
	}
}

double cDrawSceneHeadingAMP::GetTargetHeading() const
{
	const auto scene = dynamic_cast<const cSceneHeadingAMP*>(mScene.get());
	return scene->GetTargetHeading();
}

bool cDrawSceneHeadingAMP::EnableTargetPos() const
{
	const auto scene = dynamic_cast<const cSceneHeadingAMP*>(mScene.get());
	return scene->EnableTargetPos();
}

void cDrawSceneHeadingAMP::SetTargetPos(const tVector& pos)
{
	auto scene = dynamic_cast<cSceneHeadingAMP*>(mScene.get());
	scene->EnableTargetPos(true);
	
	cDrawSceneTargetAMP::SetTargetPos(pos);
}

double cDrawSceneHeadingAMP::GetTargetSpeed() const
{
	const auto scene = dynamic_cast<const cSceneHeadingAMP*>(mScene.get());
	return scene->GetTargetSpeed();
}

void cDrawSceneHeadingAMP::SetTargetSpeed(double speed)
{
	auto scene = dynamic_cast<cSceneHeadingAMP*>(mScene.get());
	scene->SetTargetSpeed(speed);
	scene->EnableRandSpeed(false);
}

void cDrawSceneHeadingAMP::ChangeTargetSpeed(double delta)
{
	double speed = GetTargetSpeed();
	speed += delta;
	SetTargetSpeed(speed);

	double new_speed = GetTargetSpeed();
	printf("Target speed: %.5f\n", new_speed);
}

void cDrawSceneHeadingAMP::DrawMisc() const
{
	cDrawSceneTargetAMP::DrawMisc();

	double heading = GetTargetHeading();
	DrawTargetHeading(heading);
}

void cDrawSceneHeadingAMP::DrawTargetPos(const tVector& target_pos) const
{
	if (EnableTargetPos())
	{
		cDrawSceneTargetAMP::DrawTargetPos(target_pos);
	}
}

void cDrawSceneHeadingAMP::DrawTargetHeading(double heading) const
{
	const tVector arrow_col = tVector(0, 0.75, 0, 0.75);
	const double speed_scale = 0.2;
	const double head_size = 0.35;
	const double y_offset = 0.025;
	const double start_offset = 0.25;

	const auto& character = mScene->GetCharacter();
	tVector com = character->CalcCOM();

	tVector start = tVector(com[0], -com[2], 0, 0);
	tVector dir = tVector(std::cos(heading), std::sin(heading), 0, 0);
	start += start_offset * dir;

	double ground_h = mScene->GetGround()->SampleHeight(com);
	start[2] += ground_h + y_offset;

	double arrow_len = speed_scale * GetTargetSpeed() + head_size;
	tVector end = start + arrow_len * dir;

	cDrawUtil::SetColor(arrow_col);
	cDrawUtil::PushMatrixView();
	cDrawUtil::Rotate(tVector(0.5 * -M_PI, 0, 0, 0));
	cDrawUtil::DrawArrow2D(start, end, head_size);
	cDrawUtil::PopMatrixView();
}