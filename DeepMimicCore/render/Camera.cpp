#include "Camera.h"
#include <iostream>

#include "render/DrawUtil.h"
#include <GL/freeglut.h>

cCamera::cCamera()
{
	mPosition = tVector(0, 0, 1, 0);
	mFocusDelta = -mPosition;
	mUp = tVector(0, 1, 0, 0);
	mNearZ = 0;
	mFarZ = 1;

	mProj = eProjPerspective;

	mMouseDown = 0;
	mMousePos = tVector::Zero();

	Resize(1, 1);
}

cCamera::~cCamera()
{
}

cCamera::cCamera(eProj proj, const tVector& pos, const tVector& focus,
	const tVector& up, double w, double h, double near_z, double far_z)
{
	mPosition = pos;
	mFocusDelta = focus - pos;
	mUp = up;
	mNearZ = near_z;
	mFarZ = far_z;
	mProj = proj;

	Resize(w, h);

	mMouseDown = 0;
	mMousePos = tVector::Zero();
}

const tVector& cCamera::GetPosition() const
{
	return mPosition;
}

tVector cCamera::GetFocus() const
{
	return mFocusDelta + mPosition;
}

const tVector& cCamera::GetUp() const
{
	return mUp;
}

tVector cCamera::GetViewDir() const
{
	tVector dir = mFocusDelta;
	dir[3] = 0;
	dir = dir.normalized();
	return dir;
}


double cCamera::GetWidth() const
{
	return mWidth;
}
double cCamera::GetHeight() const
{
	return mWidth / mAspectRatio;
}

double cCamera::GetAspectRatio() const
{
	return mAspectRatio;
}


double cCamera::GetNearZ() const
{
	return mNearZ;
}

double cCamera::GetFarZ() const
{
	return mFarZ;
}

double cCamera::CalcFOV() const
{
	double focal_len = CalcFocalLen();
	double h = GetHeight();
	double fov = std::atan(h * 0.5 / focal_len) * 2;
	return fov;
}


void cCamera::SetPosition(const tVector& pos)
{
	tVector focus = GetFocus();
	mPosition = pos;
	mFocusDelta = focus - mPosition;
}

void cCamera::SetFocus(const tVector& focus)
{
	mFocusDelta = focus - mPosition;
}

void cCamera::SetUp(const tVector& up)
{
	mUp = up;
}

void cCamera::SetNearZ(double near_z)
{
	assert(near_z < mFarZ);
	mNearZ = near_z;
}

void cCamera::SetFarZ(double far_z)
{
	assert(far_z > mNearZ);
	mFarZ = far_z;
}

void cCamera::Resize(double w, double h)
{
	mWidth = w;
	mAspectRatio = w / h;
}

void cCamera::SetViewWidth(double w)
{
	double h = w / mAspectRatio;
	Resize(w, h);
}

void cCamera::SetViewHeight(double h)
{
	double w = h * mAspectRatio;
	Resize(w, h);
}

void cCamera::TranslatePos(const tVector& pos)
{
	mPosition = pos;
}

void cCamera::TranslateFocus(const tVector& focus)
{
	mPosition = focus - mFocusDelta;
}

void cCamera::RotatePos(const tVector& axis, double theta)
{
	tQuaternion q = cMathUtil::AxisAngleToQuaternion(axis, theta);
	RotatePos(q);
}

void cCamera::RotatePos(const tQuaternion& quat)
{
	tVector pos = GetPosition() - GetFocus();
	pos = cMathUtil::QuatRotVec(quat, pos);
	pos += GetFocus();
	SetPosition(pos);
}

// screent pos normalized between [-1, 1] with positive y pointing up
tVector cCamera::ScreenToWorldPos(const tVector& screen_pos) const
{
	tMatrix T = BuildViewWorldMatrix();
	double w = GetWidth();
	double h = GetHeight();

	double focal_len = CalcFocalLen();
	tVector view_pos = tVector(0.5 * screen_pos[0] * w,
		0.5 * screen_pos[1] * h,
		-focal_len, 1);
	tVector world_pos = T * view_pos;
	world_pos[3] = 0;

	tVector dir = GetRayCastDir(world_pos);
	world_pos += -dir * (focal_len - mNearZ);

	return world_pos;
}

tVector cCamera::ProjectToFocalPlane(const tVector& world_pos) const
{
	tVector dir = GetRayCastDir(world_pos);
	tVector origin = GetFocus();
	tVector norm = GetViewDir();
	tVector delta = world_pos - origin;
	double t = (origin - world_pos).dot(norm) / dir.dot(norm);
	tVector proj_pos = t * dir + world_pos;
	return proj_pos;
}

tMatrix cCamera::BuildViewWorldMatrix() const
{
	tVector up = GetUp();
	const tVector& forward = GetViewDir();
	tVector left = up.cross3(forward).normalized();
	up = -left.cross3(forward).normalized();
	const tVector& pos = GetPosition();

	tMatrix T;
	T.col(0) = -left;
	T.col(1) = up;
	T.col(2) = -forward;
	T.col(3) = pos;
	T(3, 3) = 1;

	return T;
}

tMatrix cCamera::BuildWorldViewMatrix() const
{
	tMatrix view_world = BuildViewWorldMatrix();
	tMatrix world_view = cMathUtil::InvRigidMat(view_world);
	return world_view;
}

tMatrix cCamera::BuildProjMatrix() const
{
	tMatrix proj_mat;
	switch (mProj)
	{
	case eProjPerspective:
		proj_mat = BuildProjMatrixProj();
		break;
	case eProjOrtho:
		proj_mat = BuildProjMatrixOrtho();
		break;
	default:
		assert(false); // unsupported projection
		break;
	}
	return proj_mat;
}

void cCamera::SetProj(eProj proj)
{
	mProj = proj;
}

cCamera::eProj cCamera::GetProj() const
{
	return mProj;
}

void cCamera::SetupGLView() const
{
	tMatrix world_view = BuildWorldViewMatrix();
	cDrawUtil::LoadMatrixView(world_view);
}

void cCamera::SetupGLProj() const
{
	tMatrix proj_mat = BuildProjMatrix();
	cDrawUtil::LoadMatrixProj(proj_mat);
}

tVector cCamera::GetRayCastDir(const tVector& pos) const
{
	tVector dir = tVector::Zero();
	switch (mProj)
	{
	case eProjPerspective:
		dir = pos - mPosition;
		break;
	case eProjOrtho:
		dir = mFocusDelta;
		break;
	default:
		assert(false); // unsupported projection
		break;
	}
	dir[3] = 0;
	dir = dir.normalized();
	return dir;
}

void cCamera::MouseClick(int button, int state, double x, double y)
{
	mMouseDown = (button == GLUT_RIGHT_BUTTON) && (state == GLUT_DOWN);
	mMousePos[0] = x;
	mMousePos[1] = y;

	bool mouse_wheel = (button == 3) || (button == 4);
	if (mouse_wheel)
	{
		double zoom = (button == 3) ? -0.05 : 0.05;
		Zoom(zoom);
	}
}

void cCamera::MouseMove(double x, double y)
{
	if (mMouseDown)
	{
		int mouse_mod = glutGetModifiers();
		double w = GetWidth();
		double h = GetHeight();

		double dx = x - mMousePos[0];
		double dy = y - mMousePos[1];

		tVector focus = GetFocus();
		tVector cam_offset = -mFocusDelta;
		tVector cam_dir = -cam_offset.normalized();
		tVector right = -mUp.cross3(cam_dir).normalized();
		mUp = right.cross3(cam_dir).normalized();

		if (mouse_mod & GLUT_ACTIVE_ALT) {
			// translate
			tVector delta = 0.5 * (-w * right * dx - h * mUp * dy);
			mPosition += delta;
			focus += delta;
		}
		else
		{
			// rotate
			tMatrix rot_mat = cMathUtil::RotateMat(tVector(0, 1, 0, 0), -M_PI * dx)
				* cMathUtil::RotateMat(right, M_PI * dy);
			cam_offset = rot_mat * cam_offset;
			mUp = rot_mat * mUp;
			mPosition = focus + cam_offset;
		}

		SetFocus(focus);

		// Remember mouse coords for next time.
		mMousePos[0] = x;
		mMousePos[1] = y;
	}
}

double cCamera::CalcFocalLen() const
{
	return mFocusDelta.norm();
}

tMatrix cCamera::BuildProjMatrixOrtho() const
{
	tMatrix mat = tMatrix::Identity();
	double w = GetWidth();
	double h = GetHeight();

	mat(0, 0) = 2 / w;
	mat(1, 1) = 2 / h;
	mat(2, 2) = -2 / (mFarZ - mNearZ);
	mat(2, 3) = -(mFarZ + mNearZ) / (mFarZ - mNearZ);

	return mat;
}

tMatrix cCamera::BuildProjMatrixProj() const
{
	tMatrix mat = tMatrix::Zero();
	double focal_len = CalcFocalLen();
	double w = GetWidth();
	double h = GetHeight();

	mat(0, 0) = 2 * focal_len / w;
	mat(1, 1) = 2 * focal_len / h;
	mat(2, 2) = -(mFarZ + mNearZ) / (mFarZ - mNearZ);
	mat(2, 3) = -2 * mFarZ * mNearZ / (mFarZ - mNearZ);
	mat(3, 2) = -1;

	return mat;
}

void cCamera::Zoom(double zoom)
{
	tVector focus = GetFocus();
	tVector cam_offset = -mFocusDelta;
	double w = GetWidth();
	double h = GetHeight();

	double delta_scale = 1 - zoom;
	tVector delta = cam_offset * delta_scale;
	mPosition = focus + delta;
	w *= delta_scale;
	h *= delta_scale;
	Resize(w, h);
	SetFocus(focus);
}