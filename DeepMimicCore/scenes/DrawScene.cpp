#include "DrawScene.h"

#include "render/DrawUtil.h"

// camera attributes
const tVector gCameraPosition = tVector(0, 0, 30, 0);
const tVector gCameraFocus = tVector(gCameraPosition[0], gCameraPosition[1], 0.0, 0.0);
const tVector gCameraUp = tVector(0, 1, 0, 0);

const double gViewWidth = 4.5;
const double gViewHeight = gViewWidth * 9.0 / 16.0;
const double gViewNearZ = 2;
const double gViewFarZ = 500;

const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gVisOffset = tVector(0, 0, 1, 0); // offset for visualization elements

cDrawScene::cDrawScene()
{
	mCamTrackMode = eCamTrackModeXZ;
	mDrawInfo = true;
}

cDrawScene::~cDrawScene()
{
}

void cDrawScene::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScene::ParseArgs(parser);
	ParseCamTrackMode(parser, mCamTrackMode);
}

void cDrawScene::Init()
{
	cScene::Init();
	InitCamera();
	ResetCamera();

	InitRenderResources();
}

void cDrawScene::Reset()
{
	cScene::Reset();
	ResetCamera();
}

void cDrawScene::Draw()
{
	SetupView();
	ClearFrame();
	DrawScene();
	RestoreView();
}

void cDrawScene::MouseClick(int button, int state, double x, double y)
{
	cScene::MouseClick(button, state, x, y);
	mCamera.MouseClick(button, state, x, y);
}

void cDrawScene::MouseMove(double x, double y)
{
	cScene::MouseMove(x, y);
	mCamera.MouseMove(x, y);
}

void cDrawScene::EnableDrawInfo(bool enable)
{
	mDrawInfo = enable;
}


std::string cDrawScene::GetName() const
{
	return "Draw";
}

void cDrawScene::ParseCamTrackMode(const std::shared_ptr<cArgParser>& parser, eCamTrackMode& out_mode) const
{
	std::string str = "";
	parser->ParseString("cam_track_mode", str);

	if (str != "")
	{
		if (str == "xz")
		{
			out_mode = eCamTrackModeXZ;
		}
		else if (str == "y")
		{
			out_mode = eCamTrackModeY;
		}
		else if (str == "xyz")
		{
			out_mode = eCamTrackModeXYZ;
		}
		else if (str == "still")
		{
			out_mode = eCamTrackModeStill;
		}
		else if (str == "fixed")
		{
			out_mode = eCamTrackModeFixed;
		}
		else
		{
			assert(false); // unsupported track mode
		}
	}
}

void cDrawScene::Reshape(int w, int h)
{
	cScene::Reshape(w, h);
	ResizeCamera(w, h);
}

void cDrawScene::ResizeCamera(int w, int h)
{
	double prev_view_w = mCamera.GetWidth();
	double prev_view_h = mCamera.GetHeight();
	double new_view_h = prev_view_h;
	double new_view_w = (prev_view_h * w) / h;
	mCamera.Resize(new_view_w, new_view_h);
}

void cDrawScene::InitCamera()
{
	cCamera::eProj proj = cCamera::eProjPerspective;
	mCamera = cCamera(proj, gCameraPosition, gCameraFocus, gCameraUp,
						gViewWidth, gViewHeight, gViewNearZ, gViewFarZ);
}

cDrawScene::eCamTrackMode cDrawScene::GetCamTrackMode() const
{
	return mCamTrackMode;
}

void cDrawScene::UpdateCamera()
{
	eCamTrackMode mode = GetCamTrackMode();
	if (mode == eCamTrackModeXZ
		|| mode == eCamTrackModeY
		|| mode == eCamTrackModeXYZ)
	{
		UpdateCameraTracking();
	}
	else if (mode == eCamTrackModeStill)
	{
		UpdateCameraStill();
	}
}

void cDrawScene::UpdateCameraTracking()
{
	eCamTrackMode mode = GetCamTrackMode();
	if (mode == eCamTrackModeXYZ)
	{
		tVector track_pos = GetCamTrackPos();
		tVector focus_pos = mCamera.GetFocus();
		tVector cam_pos = mCamera.GetPosition();
		mCamera.TranslateFocus(track_pos);
	}
	else if (mode == eCamTrackModeXZ
		|| mode == eCamTrackModeY)
	{
		tVector track_pos = GetCamTrackPos();
		tVector cam_focus = mCamera.GetFocus();

		double cam_w = mCamera.GetWidth();
		double cam_h = mCamera.GetHeight();
		const double y_pad = std::min(0.5, 0.8 * 0.5 * cam_h);
		const double x_pad = std::min(0.5, 0.8 * 0.5 * cam_w);

		if (mode == eCamTrackModeXZ)
		{
			cam_focus[0] = track_pos[0];
			cam_focus[2] = track_pos[2];

			if (std::abs(track_pos[1] - cam_focus[1]) > ((0.5 * cam_h) - y_pad))
			{
				const double blend = 0.5;
				double tar_y = track_pos[1] + ((0.5 * cam_h) - y_pad);
				cam_focus[1] = (1 - blend) * cam_focus[1] + blend * tar_y;
			}
		}
		else
		{
			cam_focus[1] = track_pos[1];

			const double blend = 0.5;
			double tar_delta = track_pos[0] - cam_focus[0];
			if (std::abs(tar_delta) > ((0.5 * cam_w) - x_pad))
			{
				double tar_x = track_pos[0] + cMathUtil::Sign(tar_delta) * ((0.95 * cam_w) - x_pad);
				cam_focus[0] = (1 - blend) * cam_focus[0] + blend * tar_x;
			}
		}

		mCamera.TranslateFocus(cam_focus);
	}
}

void cDrawScene::UpdateCameraStill()
{
	tVector track_pos = GetCamTrackPos();
	tVector cam_focus = mCamera.GetFocus();

	double cam_w = mCamera.GetWidth();
	double cam_h = mCamera.GetHeight();
	double cam_still_snap_dist = GetCamStillSnapDistX();

	const double pad_x = std::min(0.5, 0.4 * cam_still_snap_dist);
	const double pad_y = std::min(0.0, 0.2 * cam_h);

	double avg_h = 0;
	bool snap_x = std::abs(track_pos[0] - cam_focus[0]) > cam_still_snap_dist - pad_x;
	bool snap_y = (track_pos[1] - cam_focus[1]) > 0.5 * cam_h - pad_y
		|| (track_pos[1] - cam_focus[1]) < -(0.5 * cam_h - pad_y);

	if (snap_x || snap_y)
	{
		tVector snap_pos = GetCamStillPos();
		cam_focus[0] = snap_pos[0];
		cam_focus[1] = snap_pos[1];

		tVector pos_delta = track_pos - snap_pos;
		if (std::abs(pos_delta[0]) > cam_still_snap_dist - pad_x)
		{
			cam_focus[0] += pos_delta[0];
		}

		if ((pos_delta[1]) > 0.5 * cam_h - pad_y
			|| (pos_delta[1]) < -(0.5 * cam_h - pad_y))
		{
			cam_focus[1] += pos_delta[1];
		}

		if (snap_x)
		{
			cam_focus[0] += cam_still_snap_dist - pad_x;
		}
	}

	mCamera.TranslateFocus(cam_focus);
}

double cDrawScene::GetCamStillSnapDistX() const
{
	const tVector axis = tVector(1, 0, 0, 0);
	double dist = 0.5 * mCamera.GetWidth();

	tVector view_delta = mCamera.GetFocus() - mCamera.GetPosition();
	tVector view_dir = view_delta.normalized();
	double len = view_delta.norm() - mCamera.GetNearZ();
	view_delta = view_dir * len;

	view_delta[1] = 0;
	view_delta[3] = 0;
	len = view_delta.norm();

	if (len > 0)
	{
		view_delta /= len;
		double dot = view_delta.dot(axis);
		double lerp = std::abs(dot);
		lerp = std::pow(lerp, 4);
		lerp = 1 - lerp;
		dist = lerp * dist + (1 - lerp) * 0.5 * len;
	}
	return dist;
}

tVector cDrawScene::GetCamTrackPos() const
{
	return tVector::Zero();
}

tVector cDrawScene::GetCamStillPos() const
{
	return tVector::Zero();
}

void cDrawScene::ResetCamera()
{
	tVector target_pos = GetDefaultCamFocus();

	eCamTrackMode mode = GetCamTrackMode();
	if (mode == eCamTrackModeXZ
		|| mode == eCamTrackModeY)
	{
		target_pos = GetCamTrackPos();
	}
	else if (mode == eCamTrackModeStill)
	{
		target_pos = GetCamStillPos();
	}

	tVector cam_pos = GetDefaultCamFocus();
	cam_pos[0] = target_pos[0];
	cam_pos[1] = target_pos[1];

	mCamera.TranslateFocus(cam_pos);
}

tVector cDrawScene::GetDefaultCamFocus() const
{
	return tVector::Zero();
}

void cDrawScene::ClearFrame()
{
	double depth = 1;
	tVector col = GetBkgColor();
	cDrawUtil::ClearDepth(depth);
	cDrawUtil::ClearColor(col);
}

tVector cDrawScene::GetBkgColor() const
{
	return tVector(0.97, 0.97, 1, 0);
}

void cDrawScene::SetupView()
{
	cDrawUtil::PushMatrixProj();
	mCamera.SetupGLProj();

	cDrawUtil::PushMatrixView();
	mCamera.SetupGLView();
}

void cDrawScene::RestoreView()
{
	cDrawUtil::PopMatrixProj();
	cDrawUtil::PopMatrixView();
}

tVector cDrawScene::GetLightDirection() const
{
	return tVector(0.6, 0.67, 0.45, 0).normalized();
}

void cDrawScene::DrawScene()
{
	DoShadowPass();
	mShaderMesh->Bind();
	SetupMeshShader();

	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);

	DrawGroundMainScene();
	DrawCharacterMainScene();
	DrawObjsMainScene();
	DrawMiscMainScene();

	mShaderMesh->Unbind();

	if (mDrawInfo)
	{
		// info is drawn in screen space
		cDrawUtil::PushMatrixProj();
		cDrawUtil::LoadIdentityProj();

		cDrawUtil::PushMatrixView();
		cDrawUtil::LoadIdentityView();

		DrawInfo();

		cDrawUtil::PopMatrixProj();
		cDrawUtil::PopMatrixView();
	}
}

void cDrawScene::DrawGrid() const
{
	const double spacing = 0.10f;
	const double line_width = 1;
	const double big_spacing = spacing * 5.f;
	const double big_line_width = 2;
	const tVector offset = tVector(0, 0, -1, 0);

	tVector origin = mCamera.GetFocus();
	origin += offset;
	tVector size = tVector(mCamera.GetWidth(), mCamera.GetHeight(), 0, 0);

	cDrawUtil::DrawGrid2D(origin, size, spacing, line_width);
	cDrawUtil::DrawGrid2D(origin, size, big_spacing, big_line_width);
}

void cDrawScene::DrawGroundMainScene()
{
	const double roughness = 0.5;
	const double enable_tex = 1;

	mShaderMesh->SetUniform4(mMeshMaterialDataHandle, tVector(roughness, enable_tex, 0, 0));
	mGridTex->BindTex(GL_TEXTURE0);

	DrawGround();

	mGridTex->UnbindTex(GL_TEXTURE0);
}

void cDrawScene::DrawCharacterMainScene()
{
	const double roughness = 0.4;
	const double enable_tex = 0;
	mShaderMesh->SetUniform4(mMeshMaterialDataHandle, tVector(roughness, enable_tex, 0, 0));
	DrawCharacters();
}

void cDrawScene::DrawObjsMainScene()
{
	const double roughness = 0.4;
	const double enable_tex = 0;
	mShaderMesh->SetUniform4(mMeshMaterialDataHandle, tVector(roughness, enable_tex, 0, 0));
	DrawObjs();
}

void cDrawScene::DrawMiscMainScene()
{
	const double roughness = 0.4;
	const double enable_tex = 0;
	mShaderMesh->SetUniform4(mMeshMaterialDataHandle, tVector(roughness, enable_tex, 0, 0));
	DrawMisc();
}

void cDrawScene::DrawGround() const
{
}

void cDrawScene::DrawCharacters() const
{
}

void cDrawScene::DrawObjs() const
{
}

void cDrawScene::DrawMisc() const
{
}

void cDrawScene::DrawInfo() const
{
}


tVector cDrawScene::GetVisOffset() const
{
	return tVector::Zero();
}

tVector cDrawScene::GetLineColor() const
{
	return gLineColor;
}

tVector cDrawScene::GetGroundColor() const
{
	return tVector::Ones();
}

void cDrawScene::InitRenderResources()
{
	bool succ = true;

	{
		mShaderDepth = std::unique_ptr<cShader>(new cShader());
		succ &= mShaderDepth->BuildShader("data/shaders/Mesh_VS.glsl", "data/shaders/Depth_PS.glsl");
	}

	{
		mShaderMesh = std::unique_ptr<cShader>(new cShader());
		succ &= mShaderMesh->BuildShader("data/shaders/Mesh_VS.glsl", "data/shaders/Lighting_Lambert_PS.glsl");

		mShaderMesh->Bind();
		mShaderMesh->GetUniformHandle(mMeshLightDirHandle, "gLightDir");
		mShaderMesh->GetUniformHandle(mMeshLightColourHandle, "gLightColour");
		mShaderMesh->GetUniformHandle(mMeshAmbientColourHandle, "gAmbientColour");
		mShaderMesh->GetUniformHandle(mMeshShadowProjHandle, "gShadowProj");
		mShaderMesh->GetUniformHandle(mMeshMaterialDataHandle, "gMaterialData");
		mShaderMesh->GetUniformHandle(mMeshFogColorHandle, "gFogColor");
		mShaderMesh->GetUniformHandle(mMeshFogDataHandle, "gFogData");

		GLint albedo_tex = glGetUniformLocation(mShaderMesh->GetProg(), "gTexture");
		glUniform1i(albedo_tex, 0);
		GLint shadow_tex = glGetUniformLocation(mShaderMesh->GetProg(), "gShadowTex");
		glUniform1i(shadow_tex, 1);
		mShaderMesh->Unbind();
	}

	float shadow_size = 40.f;
	float shadow_near_z = 1.f;
	float shadow_far_z = 60.f;
	int shadow_res = 2048;
	mShadowCam = cCamera(cCamera::eProjOrtho, tVector(0, 0, 1, 0), tVector::Zero(),
		tVector(0, 1, 0, 0), shadow_size, shadow_size, shadow_near_z,
		shadow_far_z);
	mShadowMap = std::unique_ptr<cShadowMap>(new cShadowMap());
	mShadowMap->Init(shadow_res, shadow_res);

	succ &= LoadTextures();

	if (!succ)
	{
		printf("Failed to setup render resources\n");
	}
}

bool cDrawScene::LoadTextures()
{
	bool succ = true;
	mGridTex = std::unique_ptr<cTextureDesc>(new cTextureDesc("data/textures/grid0.png", true));
	succ &= mGridTex->IsValid();
	return succ;
}

void cDrawScene::SetupMeshShader()
{
	tMatrix view_mat = mCamera.BuildWorldViewMatrix();

	tVector cam_focus = mCamera.GetFocus();
	tVector cam_pos = mCamera.GetPosition();
	double near_dist = mCamera.GetNearZ();

	const double fog_cutoff = (cam_focus - cam_pos).norm() - near_dist;
	const double fog_decay = 0.001;
	const tVector ambient_col = tVector(0.6, 0.6, 0.6, 0);
	const tVector light_col = tVector(0.5, 0.5, 0.5, 0);
	const tVector fog_col = tVector(0.97, 0.97, 1, 1);
	const tVector fog_data = tVector(fog_cutoff, fog_decay, 0, 0);

	tVector light_dir = GetLightDirection();
	light_dir = view_mat * light_dir;

	mShaderMesh->SetUniform3(mMeshLightDirHandle, light_dir);
	mShaderMesh->SetUniform3(mMeshLightColourHandle, light_col);
	mShaderMesh->SetUniform3(mMeshAmbientColourHandle, ambient_col);
	mShaderMesh->SetUniform4(mMeshFogColorHandle, fog_col);
	mShaderMesh->SetUniform4(mMeshFogDataHandle, fog_data);

	tMatrix view_world = mCamera.BuildViewWorldMatrix();
	tMatrix shadow_view = mShadowCam.BuildWorldViewMatrix();
	tMatrix shadow_proj = mShadowCam.BuildProjMatrix();
	tMatrix shadow_mat = shadow_proj * shadow_view * view_world;

	float shadow_mat_data[] = { (float)shadow_mat(0, 0), (float)shadow_mat(1, 0), (float)shadow_mat(2, 0), (float)shadow_mat(3, 0),
		(float)shadow_mat(0, 1), (float)shadow_mat(1, 1), (float)shadow_mat(2, 1), (float)shadow_mat(3, 1),
		(float)shadow_mat(0, 2), (float)shadow_mat(1, 2), (float)shadow_mat(2, 2), (float)shadow_mat(3, 2),
		(float)shadow_mat(0, 3), (float)shadow_mat(1, 3), (float)shadow_mat(2, 3), (float)shadow_mat(3, 3) };
	glProgramUniformMatrix4fv(mShaderMesh->GetProg(), mMeshShadowProjHandle, 1, false, shadow_mat_data);

	mShadowMap->BindTex(GL_TEXTURE1);
}

void cDrawScene::DoShadowPass()
{
	// front face culling to prevent shelf occlusion
	glCullFace(GL_FRONT);

	float dist = 30.f;
	const tVector& sun_dir = GetLightDirection();
	tVector delta = dist * sun_dir;
	tVector focus = mCamera.GetFocus();
	mShadowCam.SetPosition(focus + delta);
	mShadowCam.SetFocus(focus);

	mShadowMap->BindBuffer();
	mShaderDepth->Bind();
	cDrawUtil::ClearColor(tVector(1, 1, 1, 0));
	cDrawUtil::ClearDepth(1);

	// shadow pass
	cDrawUtil::PushMatrixProj();
	mShadowCam.SetupGLProj();

	cDrawUtil::PushMatrixView();
	mShadowCam.SetupGLView();

	DrawCharacters();
	DrawObjs();
	DrawMisc();
	glCullFace(GL_BACK);
	DrawGround();

	mShaderDepth->Unbind();
	mShadowMap->UnbindBuffer();

	cDrawUtil::PopMatrixProj();
	cDrawUtil::PopMatrixView();
}
