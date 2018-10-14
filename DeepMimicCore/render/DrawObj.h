#pragma once

#include "DrawUtil.h"
#include "util/MathUtil.h"
#include "sim/SimObj.h"

class cDrawObj
{
public:
	static void Draw(const cSimObj* obj, cDrawUtil::eDrawMode draw_mode = cDrawUtil::eDrawSolid);
	static void DrawBox(const cSimObj* box, cDrawUtil::eDrawMode draw_mode = cDrawUtil::eDrawSolid);
	static void DrawBox(const cSimObj* box, const tVector& tex_coord_min, const tVector& tex_coord_max, cDrawUtil::eDrawMode draw_mode = cDrawUtil::eDrawSolid);
	static void DrawPlane(const cSimObj* plane, double size, cDrawUtil::eDrawMode draw_mode = cDrawUtil::eDrawSolid);
	static void DrawCapsule(const cSimObj* cap, cDrawUtil::eDrawMode draw_mode = cDrawUtil::eDrawSolid);
	static void DrawSphere(const cSimObj* box, cDrawUtil::eDrawMode draw_mode = cDrawUtil::eDrawSolid);
	static void DrawCylinder(const cSimObj* cylinder, cDrawUtil::eDrawMode draw_mode = cDrawUtil::eDrawSolid);
};