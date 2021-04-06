#include "DrawSceneSimChar.h"

#include "sim/SimBox.h"

#include "render/DrawUtil.h"
#include "render/DrawGround.h"
#include "render/DrawCharacter.h"
#include "render/DrawSimCharacter.h"
#include "render/DrawObj.h"
#include "render/DrawWorld.h"
#include "render/DrawPerturb.h"

const tVector gCamFocus0 = tVector(0, 0.75, 0, 0);

const tVector gLineColor = tVector(0, 0, 0, 1);
const size_t gInitGroundUpdateCount = std::numeric_limits<size_t>::max();

const std::string gOutputCharFile = "output/char_state.txt";

cDrawSceneSimChar::cDrawSceneSimChar()
{
	mEnableTrace = false;
	mTracerBufferSize = 2000;
	mTracerSamplePeriod = 1 / 60.0;
	ResetUI();
}

cDrawSceneSimChar::~cDrawSceneSimChar()
{
}

void cDrawSceneSimChar::Init()
{
	BuildScene(mScene);
	SetupScene(mScene);
	
	cDrawScene::Init();

	InitTracer();
	BuildGroundDrawMesh();
}

void cDrawSceneSimChar::Clear()
{
	cDrawScene::Clear();
	mScene->Clear();
	mTracer.Clear();
	mPrevGroundUpdateCount = gInitGroundUpdateCount;
}

void cDrawSceneSimChar::Update(double time_elapsed)
{
	cDrawScene::Update(time_elapsed);

	UpdateScene(time_elapsed);
	if (mEnableTrace)
	{
		UpdateTracer(time_elapsed);
	}
	
	UpdateGroundDrawMesh();
	UpdateCamera();
}

const std::shared_ptr<cSceneSimChar>& cDrawSceneSimChar::GetScene() const
{
	return mScene;
}

void cDrawSceneSimChar::MouseClick(int button, int state, double x, double y)
{
	const double ray_max_dist = 1000;
	cDrawScene::MouseClick(button, state, x, y);

	if (button == GLUT_LEFT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			mClickScreenPos = tVector(x, y, 0, 0);
			mDragScreenPos = mClickScreenPos;
			tVector start = mCamera.ScreenToWorldPos(mClickScreenPos);
			tVector dir = mCamera.GetRayCastDir(start);
			tVector end = start + dir * ray_max_dist;

			cWorld::tRayTestResult raytest_result;
			RayTest(start, end, raytest_result);
			HandleRayTest(raytest_result);
		}
		else if (state == GLUT_UP)
		{
			ResetUI();
		}
	}
}

void cDrawSceneSimChar::MouseMove(double x, double y)
{
	cDrawScene::MouseMove(x, y);

	if (ObjectSelected())
	{
		mDragScreenPos = tVector(x, y, 0, 0);
	}
}

void cDrawSceneSimChar::Keyboard(unsigned char key, double device_x, double device_y)
{
	cDrawScene::Keyboard(key, device_x, device_y);

	switch (key)
	{
	case 's':
		OutputCharState(GetOutputCharFile());
		break;
	case 'x':
		SpawnProjectile();
		break;
	case 'z':
		SpawnBigProjectile();
		break;
	case 'y':
		ToggleTrace();
		break;
	default:
		break;
	}
}

double cDrawSceneSimChar::GetTime() const
{
	return mScene->GetTime();
}

bool cDrawSceneSimChar::IsEpisodeEnd() const
{
	return mScene->IsEpisodeEnd();
}

bool cDrawSceneSimChar::CheckValidEpisode() const
{
	return mScene->CheckValidEpisode();
}

std::string cDrawSceneSimChar::GetName() const
{
	return mScene->GetName();
}

void cDrawSceneSimChar::BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cSceneSimChar>(new cSceneSimChar());
}

void cDrawSceneSimChar::SetupScene(std::shared_ptr<cSceneSimChar>& out_scene)
{
	out_scene->ParseArgs(mArgParser);
	out_scene->Init();
}

void cDrawSceneSimChar::UpdateScene(double time_elapsed)
{
	ApplyUIForce(time_elapsed);
	mScene->Update(time_elapsed);
}

void cDrawSceneSimChar::ResetScene()
{
	cDrawScene::ResetScene();
	mScene->Reset();
	mTracer.Reset();
	BuildGroundDrawMesh();
}

tVector cDrawSceneSimChar::GetCamTrackPos() const
{
	const auto& character = mScene->GetCharacter();
	return character->CalcCOM();
}

tVector cDrawSceneSimChar::GetCamStillPos() const
{
	const auto& character = mScene->GetCharacter();
	tVector char_pos = character->CalcCOM();

	double cam_w = mCamera.GetWidth();
	double cam_h = mCamera.GetHeight();
	const auto& ground = mScene->GetGround();

	const int num_samples = 16;
	double ground_samples[num_samples] = { 0 };
	const double pad = std::min(0.5, 0.5 * cam_w);

	double avg_h = 0;

	double min_x = char_pos[0];
	double max_x = char_pos[0] + cam_w;

	int num_valid_samples = 0;
	for (int i = 0; i < num_samples; ++i)
	{
		tVector pos = char_pos;
		pos[0] = static_cast<double>(i) / (num_samples - 1) * (max_x - min_x) + min_x;

		bool valid_sample = true;
		double ground_h = ground->SampleHeight(pos, valid_sample);
		if (valid_sample)
		{
			ground_samples[i] = ground_h;
			avg_h += ground_h;
			++num_valid_samples;
		}
	}
	avg_h /= num_valid_samples;

	std::sort(ground_samples, &(ground_samples[num_samples - 1]));
	double med_h = ground_samples[num_samples / 2];
	double min_h = ground_samples[0];

	tVector track_pos = char_pos;
	double target_h = avg_h;
	
	double y_pad = -0.4;
	track_pos[1] = target_h + y_pad + 0.5 * cam_h;

	return track_pos;
}

tVector cDrawSceneSimChar::GetDefaultCamFocus() const
{
	return gCamFocus0;
}

void cDrawSceneSimChar::ResetParams()
{
	cDrawScene::ResetParams();
	ResetUI();
}

void cDrawSceneSimChar::ToggleTrace()
{
	mTracer.Reset();
	mEnableTrace = !mEnableTrace;
	if (mEnableTrace)
	{
		printf("Enable character tracer\n");
	}
	else
	{
		printf("Disable character tracer\n");
	}
}

void cDrawSceneSimChar::InitTracer()
{
	mTraceHandles.clear();
	mTracer.Init(mTracerSamplePeriod);
	AddTraces();
}

void cDrawSceneSimChar::AddTraces()
{
	tVectorArr tracer_cols;
	tracer_cols.push_back(tVector(0, 0, 1, 0.5));
	tracer_cols.push_back(tVector(1, 0, 0, 0.5));
	tracer_cols.push_back(tVector(0, 0.5, 0, 0.5));
	tracer_cols.push_back(tVector(0.75, 0, 0.75, 0.5));
	tracer_cols.push_back(tVector(0, 0.5, 0.5, 0.5));
	tracer_cols.push_back(tVector(0, 0, 0, 0.5));
	
	for (int i = 0; i < mScene->GetNumChars(); ++i)
	{
		AddCharTrace(mScene->GetCharacter(i), tracer_cols);
	}
}

void cDrawSceneSimChar::AddCharTrace(const std::shared_ptr<cSimCharacter>& character,
									const tVectorArr& cols)
{
	cObjTracer::tParams com_params;
	com_params.mObj = character;
	com_params.mColors = cols;
	com_params.mType = cObjTracer::eTraceCOM;
	com_params.mBufferSize = mTracerBufferSize;

	int com_handle = mTracer.AddTrace(com_params);
	mTraceHandles.push_back(com_handle);

	int end_eff_idx = 0;
	for (int i = 0; i < character->GetNumBodyParts(); ++i)
	{
		if (character->IsValidBodyPart(i)
			&& character->IsEndEffector(i))
		{
			cObjTracer::tParams contact_params;
			contact_params.mObj = character->GetBodyPart(i);
			contact_params.mColors = cols;
			contact_params.mType = cObjTracer::eTraceContact;
			contact_params.mShapeIdx = end_eff_idx;
			contact_params.mBufferSize = mTracerBufferSize / 10;

			int contact_handle = mTracer.AddTrace(contact_params);
			mTraceHandles.push_back(contact_handle);
			++end_eff_idx;
		}
	}
}

void cDrawSceneSimChar::UpdateTracer(double time_elapsed)
{
	mTracer.Update(time_elapsed);
}

void cDrawSceneSimChar::SpawnProjectile()
{
	mScene->SpawnProjectile();
}

void cDrawSceneSimChar::SpawnBigProjectile()
{
	mScene->SpawnBigProjectile();
}

void cDrawSceneSimChar::OutputCharState(const std::string& out_file) const
{
	mScene->OutputCharState(out_file);
}

std::string cDrawSceneSimChar::GetOutputCharFile() const
{
	return gOutputCharFile;
}


void cDrawSceneSimChar::ResetUI()
{
	mClickScreenPos.setZero();
	mDragScreenPos.setZero();
	mSelectObjLocalPos.setZero();
	mSelectedObj = nullptr;
}

void cDrawSceneSimChar::RayTest(const tVector& start, const tVector& end, cWorld::tRayTestResult& out_result)
{
	return GetScene()->RayTest(start, end, out_result);
}

bool cDrawSceneSimChar::ObjectSelected() const
{
	return mSelectedObj != nullptr;
}

void cDrawSceneSimChar::HandleRayTest(const cWorld::tRayTestResult& result)
{
	if (result.mObj != nullptr)
	{
		cSimObj::eType obj_type = result.mObj->GetType();
		if (obj_type == cSimObj::eTypeDynamic)
		{
			mSelectedObj = result.mObj;
			if (ObjectSelected())
			{
				mSelectObjLocalPos = mSelectedObj->WorldToLocalPos(result.mHitPos);
			}
		}
	}
}

void cDrawSceneSimChar::ApplyUIForce(double time_step)
{
	if (ObjectSelected())
	{
		const double force_scale = 1 / cDrawPerturb::gForceScale;
		tVector start = mCamera.ScreenToWorldPos(mClickScreenPos);
		tVector end = mCamera.ScreenToWorldPos(mDragScreenPos);
		start = mCamera.ProjectToFocalPlane(start);
		end = mCamera.ProjectToFocalPlane(end);

		tVector force = end - start;
		force *= force_scale;

		tPerturb perturb = tPerturb(tPerturb::ePerturbForce, mSelectedObj, mSelectObjLocalPos,
									force, time_step);
		GetScene()->AddPerturb(perturb);
	}
}

void cDrawSceneSimChar::DrawObjs() const
{
	cDrawUtil::SetLineWidth(1.0);

	int num_objs = mScene->GetNumObjs();
	for (int i = 0; i < num_objs; ++i)
	{
		DrawObj(i);
	}
}

void cDrawSceneSimChar::DrawObj(int obj_id) const
{
	const cSceneSimChar::tObjEntry& entry = mScene->GetObjEntry(obj_id);
	if (entry.IsValid())
	{
		const auto& obj = entry.mObj;
		cDrawUtil::SetColor(entry.mColor);
		cDrawObj::Draw(obj.get(), cDrawUtil::eDrawSolid);

		tVector line_col = GetLineColor();
		if (line_col[3] > 0)
		{
			cDrawUtil::SetColor(line_col);
			cDrawObj::Draw(obj.get(), cDrawUtil::eDrawWireSimple);
		}
	}
}

void cDrawSceneSimChar::DrawMisc() const
{
	if (mEnableTrace)
	{
		DrawTrace();
	}
	DrawPerturbs();
}

void cDrawSceneSimChar::DrawCoM() const
{
	const tVector col = tVector(0, 0.8, 0, 0.5);
	const double marker_size = 0.1;
	const double vel_scale = 0.1;

	for (int i = 0; i < mScene->GetNumChars(); ++i)
	{
		const auto& character = mScene->GetCharacter(i);
		cDrawSimCharacter::DrawCoM(*(character.get()), marker_size, vel_scale, col, GetVisOffset());
	}
}

void cDrawSceneSimChar::DrawTorque() const
{
	for (int i = 0; i < mScene->GetNumChars(); ++i)
	{
		const auto& character = mScene->GetCharacter(i);
		cDrawSimCharacter::DrawTorque(*(character.get()), GetVisOffset());
	}
}

void cDrawSceneSimChar::DrawBodyVel() const
{
	const double lin_vel_scale = 0.1;
	const double ang_vel_scale = 1 / (2 * M_PI);
	for (int i = 0; i < mScene->GetNumChars(); ++i)
	{
		const auto& character = mScene->GetCharacter(i);
		cDrawSimCharacter::DrawBodyVel(*(character.get()), lin_vel_scale, ang_vel_scale, GetVisOffset());
	}
}

void cDrawSceneSimChar::DrawHeading() const
{
	for (int i = 0; i < mScene->GetNumChars(); ++i)
	{
		const auto& character = mScene->GetCharacter(i);
		double arrow_size = 0.2;
		tVector arrow_col = tVector(0, 0.8, 0, 0.5);
		cDrawCharacter::DrawHeading(*(character.get()), arrow_size, arrow_col, GetVisOffset());
	}
}

void cDrawSceneSimChar::DrawTrace() const
{
	cDrawUtil::PushMatrixView();
	cDrawUtil::Translate(GetVisOffset());
	mTracer.Draw();
	cDrawUtil::PopMatrixView();
}


void cDrawSceneSimChar::DrawPerturbs() const
{
	const auto& world = mScene->GetWorld();
	cDrawWorld::DrawPerturbs(*world.get());
}

void cDrawSceneSimChar::DrawGround() const
{
	const auto& ground = mScene->GetGround();

	tVector focus = mCamera.GetFocus();
	double cam_w = mCamera.GetWidth();
	double cam_h = mCamera.GetHeight();

	tVector ground_col = GetGroundColor();
	cDrawUtil::SetColor(ground_col);
	mGroundDrawMesh->Draw();
}

void cDrawSceneSimChar::DrawCharacters() const
{
	int num_chars = mScene->GetNumChars();
	for (int i = 0; i < num_chars; ++i)
	{
		const auto& curr_char = mScene->GetCharacter(i);
		DrawCharacter(curr_char);
	}
}

void cDrawSceneSimChar::DrawCharacter(const std::shared_ptr<cSimCharacter>& character) const
{
	const tVector fill_tint = tVector(1, 1, 1, 1);
	bool enable_draw_shape = true;
	cDrawSimCharacter::Draw(*(character.get()), fill_tint, GetLineColor(), enable_draw_shape);
}

void cDrawSceneSimChar::UpdateGroundDrawMesh()
{
	const auto& ground = mScene->GetGround();
	size_t update_count = ground->GetUpdateCount();
	if (update_count != mPrevGroundUpdateCount)
	{
		const auto& ground = mScene->GetGround();
		cDrawGround::BuildMesh(ground.get(), mGroundDrawMesh.get());
		mPrevGroundUpdateCount = ground->GetUpdateCount();
	}
}

void cDrawSceneSimChar::BuildGroundDrawMesh()
{
	mGroundDrawMesh = std::unique_ptr<cDrawMesh>(new cDrawMesh());
	mGroundDrawMesh->Init(1);

	const auto& ground = mScene->GetGround();
	cDrawGround::BuildMesh(ground.get(), mGroundDrawMesh.get());
	mPrevGroundUpdateCount = ground->GetUpdateCount();
}

void cDrawSceneSimChar::DrawInfo() const
{
	DrawPoliInfo();
}

void cDrawSceneSimChar::DrawPoliInfo() const
{
	const auto& character = mScene->GetCharacter();
	const cDeepMimicCharController* char_ctrl = dynamic_cast<cDeepMimicCharController*>(character->GetController().get());
	if (char_ctrl != nullptr)
	{
		cDrawSimCharacter::DrawInfoValLog(char_ctrl->GetValLog(), mCamera);
	}
}