#include "DrawSceneTargetAMP.h"
#include "SceneTargetAMP.h"
#include "render/DrawUtil.h"

cDrawSceneTargetAMP::cDrawSceneTargetAMP()
{
}

cDrawSceneTargetAMP::~cDrawSceneTargetAMP()
{
}

void cDrawSceneTargetAMP::Reset()
{
	auto scene = dynamic_cast<cSceneTargetAMP*>(mScene.get());
	scene->EnableRandTargetPos(true);

	cDrawRLScene::Reset();
}

void cDrawSceneTargetAMP::BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cSceneTargetAMP>(new cSceneTargetAMP());
}

tVector cDrawSceneTargetAMP::GetTargetPos() const
{
	const auto scene = dynamic_cast<const cSceneTargetAMP*>(mScene.get());
	return scene->GetTargetPos();
}

void cDrawSceneTargetAMP::SetTargetPos(const tVector& pos)
{
	auto scene = dynamic_cast<cSceneTargetAMP*>(mScene.get());
	scene->SetTargetPos(pos);
	scene->EnableRandTargetPos(false);
}

bool cDrawSceneTargetAMP::CheckTargetSucc() const
{
	auto scene = dynamic_cast<const cSceneTargetAMP*>(mScene.get());
	bool succ = scene->CheckTargetSucc();
	return succ;
}

void cDrawSceneTargetAMP::HandleRayTest(const cWorld::tRayTestResult& result)
{
	cDrawSceneSimChar::HandleRayTest(result);

	if (result.mObj != nullptr)
	{
		cSimObj::eType obj_type = result.mObj->GetType();
		if (obj_type == cSimObj::eTypeStatic)
		{
			SetTargetPos(result.mHitPos);
		}
	}
}

void cDrawSceneTargetAMP::DrawMisc() const
{
	cDrawSceneSimChar::DrawMisc();

	const tVector& target_pos = GetTargetPos();
	DrawTargetPos(target_pos);
}

void cDrawSceneTargetAMP::DrawTargetPos(const tVector& target_pos) const
{
	const double pole_r = 0.025;
	const double pole_h = 2.5;
	const double base_r = 0.1;
	const double base_h = 0.05;
	const double top_r = 0.05;
	const double flag_w = 0.5;
	const double flag_h = flag_w / 1.618;
	const double flag_d = 0.01;

	const tVector line_col = tVector(0, 0, 0, 1.0);
	const tVector pole_col = tVector(0.5, 0.5, 0.5, 1.0);
	const tVector flag_col0 = tVector(1, 0, 0, 0.5);
	const tVector flag_col1 = tVector(0, 0.75, 0, 0.5);

	auto scene = dynamic_cast<const cSceneTargetAMP*>(mScene.get());
	double succ_dist = scene->GetTargetSuccDist();

	bool succ = CheckTargetSucc();
	tVector flag_col = flag_col0;
	if (succ)
	{
		flag_col = flag_col1;
	}

	cDrawUtil::SetLineWidth(1);
	cDrawUtil::PushMatrixView();

	cDrawUtil::Translate(target_pos);

	cDrawUtil::SetLineWidth(3);
	cDrawUtil::SetColor(flag_col);
	cDrawUtil::Translate(tVector(0, 0.1, 0, 0));
	cDrawUtil::Rotate(0.5 * M_PI, tVector(1, 0, 0, 0));
	cDrawUtil::DrawDisk(succ_dist, cDrawUtil::eDrawWireSimple);
	cDrawUtil::Rotate(-0.5 * M_PI, tVector(1, 0, 0, 0));
	cDrawUtil::Translate(tVector(0, -0.1, 0, 0));


	cDrawUtil::SetLineWidth(1);
	cDrawUtil::Translate(tVector(0, 0.5 * base_h, 0, 0));
	cDrawUtil::SetColor(pole_col);
	cDrawUtil::DrawCylinder(base_r, base_h);
	cDrawUtil::SetColor(line_col);
	cDrawUtil::DrawCylinder(base_r, base_h, cDrawUtil::eDrawWireSimple);

	cDrawUtil::Translate(tVector(0, 0.5 * pole_h - 0.5 * base_h, 0, 0));
	cDrawUtil::SetColor(pole_col);
	cDrawUtil::DrawCylinder(pole_r, pole_h);
	cDrawUtil::SetColor(line_col);
	cDrawUtil::DrawCylinder(pole_r, pole_h, cDrawUtil::eDrawWireSimple);
	
	cDrawUtil::Translate(tVector(0, 0.5 * pole_h, 0, 0));
	cDrawUtil::SetColor(pole_col);
	cDrawUtil::DrawSphere(top_r);
	cDrawUtil::SetColor(line_col);
	cDrawUtil::DrawSphere(top_r, cDrawUtil::eDrawWireSimple);


	cDrawUtil::SetColor(flag_col);
	cDrawUtil::DrawBox(tVector(0.5 * flag_w + pole_r, -0.5 * flag_h - top_r, 0, 0),
						tVector(flag_w, flag_h, flag_d, 0));
	cDrawUtil::SetColor(line_col);
	cDrawUtil::DrawBox(tVector(0.5 * flag_w + pole_r, -0.5 * flag_h - top_r, 0, 0),
						tVector(flag_w, flag_h, flag_d, 0), cDrawUtil::eDrawWireSimple);

	cDrawUtil::PopMatrixView();
}