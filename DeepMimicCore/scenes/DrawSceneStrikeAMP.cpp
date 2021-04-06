#include "DrawSceneStrikeAMP.h"
#include "scenes/SceneStrikeAMP.h"
#include "render/DrawUtil.h"

cDrawSceneStrikeAMP::cDrawSceneStrikeAMP() : cDrawSceneTargetAMP()
{
	mTracerBufferSize = 200;
}

cDrawSceneStrikeAMP::~cDrawSceneStrikeAMP()
{
}

void cDrawSceneStrikeAMP::BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cSceneStrikeAMP>(new cSceneStrikeAMP());
}

void cDrawSceneStrikeAMP::AddTraces()
{
	const auto scene = dynamic_cast<const cSceneStrikeAMP*>(mScene.get());
	const auto& strike_bodies = scene->GetStrikeBodies();
	for (int i = 0; i < scene->GetNumChars(); ++i)
	{
		for (int j = 0; j < static_cast<int>(strike_bodies.size()); ++j)
		{
			const auto& sim_char = mScene->GetCharacter(i);
			cObjTracer::tParams trace_params;
			trace_params.mObj = sim_char->GetBodyPart(strike_bodies[j]);
			trace_params.mColors.push_back(tVector(0, 0, 1, 0.5));
			trace_params.mType = cObjTracer::eTracePos;
			trace_params.mBufferSize = mTracerBufferSize;

			int handle = mTracer.AddTrace(trace_params);
			mTraceHandles.push_back(handle);
		}
	}
}

void cDrawSceneStrikeAMP::DrawTargetPos(const tVector& target_pos) const
{
	const tVector no_hit_col = tVector(1, 0, 0, 0.5);
	const tVector hit_col = tVector(0, 0.75, 0, 0.5);

	auto scene = dynamic_cast<const cSceneStrikeAMP*>(mScene.get());
	bool hit = scene->TargetHit();
	double r = scene->GetTargetRadius();

	tVector col;
	if (hit)
	{
		col = hit_col;
	}
	else
	{
		col = no_hit_col;
	}

	cDrawUtil::PushMatrixView();
	cDrawUtil::Translate(target_pos);
	cDrawUtil::SetColor(col);
	cDrawUtil::DrawSphere(r);
	cDrawUtil::PopMatrixView();


	const auto& ground = scene->GetGround();
	double ground_h = ground->SampleHeight(target_pos);
	const double near_dist = scene->GetTarNearDist();

	cDrawUtil::PushMatrixView();

	cDrawUtil::SetLineWidth(3);
	cDrawUtil::SetColor(col);
	cDrawUtil::Translate(tVector(target_pos[0], ground_h + 0.1, target_pos[2], 0));
	cDrawUtil::Rotate(0.5 * M_PI, tVector(1, 0, 0, 0));
	cDrawUtil::DrawDisk(near_dist, cDrawUtil::eDrawWireSimple);

	cDrawUtil::PopMatrixView();
}

void cDrawSceneStrikeAMP::SetTargetPos(const tVector& pos)
{
	auto scene = dynamic_cast<cSceneStrikeAMP*>(mScene.get());
	scene->SetTargetPosXZ(pos);
	scene->SetTargetHit(false);
}