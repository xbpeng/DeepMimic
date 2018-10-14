#pragma once

#include <memory>
#include "util/MathUtil.h"
#include "render/TextureDesc.h"
#include "render/DrawMesh.h"
#include "render/Shader.h"
#include "render/MeshUtil.h"
#include "render/MatrixStack.h"

class cDrawUtil
{
public:
	enum eDrawMode
	{
		eDrawSolid,
		eDrawWire,
		eDrawWireSimple,
		eDrawMax
	};

	static void EnableDraw(bool enable);
	static bool EnableDraw();

	static void InitOffscreenDrawContext();
	static bool CheckDrawContextInit();
	static void InitDrawUtil();

	static void DrawRect(const tVector& pos, const tVector& size, eDrawMode draw_mode = eDrawSolid);
	static void DrawBox(const tVector& pos, const tVector& size, eDrawMode draw_mode = eDrawSolid);
	static void DrawBox(const tVector& pos, const tVector& size, const tVector& tex_coord_min, const tVector& tex_coord_max, eDrawMode draw_mode = eDrawSolid);
	static void DrawTriangle(const tVector& pos, double side_len, eDrawMode draw_mode = eDrawSolid);
	static void DrawQuad(const tVector& a, const tVector& b, const tVector& c, const tVector& d, eDrawMode draw_mode = eDrawSolid);
	static void DrawQuad(const tVector& a, const tVector& b, const tVector& c, const tVector& d,
		const tVector& coord_a, const tVector& coord_b, const tVector& coord_c, const tVector& coord_d,
		eDrawMode draw_mode = eDrawSolid);
	static void DrawDisk(const tVector& pos, double r, eDrawMode draw_mode = eDrawSolid);
	static void DrawDisk(const tVector& pos, const tVector& r, eDrawMode draw_mode = eDrawSolid);
	static void DrawDisk(double r, eDrawMode draw_mode = eDrawSolid);
	static void DrawDisk(const tVector& r, eDrawMode draw_mode = eDrawSolid);
	static void DrawPoint(const tVector& pt);
	static void DrawLine(const tVector& a, const tVector& b);
	static void DrawLineStrip(const tVectorArr& pts);
	static void DrawStrip(const tVector& a, const tVector& b, double width, eDrawMode draw_mode = eDrawSolid);
	static void DrawCross(const tVector& pos, double size);
	static void DrawPlane(const tVector& coeffs, double size, eDrawMode draw_mode = eDrawSolid);
	static void DrawSphere(double r, eDrawMode draw_mode = eDrawSolid);
	static void DrawHemisphere(double r, eDrawMode draw_mode = eDrawSolid);
	static void DrawCylinder(double r, double h, eDrawMode draw_mode = eDrawSolid);
	static void DrawCone(double r, double h, eDrawMode draw_mode = eDrawSolid);
	static void DrawTube(double r, double h, eDrawMode draw_mode = eDrawSolid);
	static void DrawCapsule(double r, double h, eDrawMode draw_mode = eDrawSolid);

	static void DrawArrow2D(const tVector& start, const tVector& end, double head_size);
	static void DrawArrow3D(const tVector& start, const tVector& end, double head_size);
	static void DrawGrid2D(const tVector& origin, const tVector& size, double spacing, double line_width);
	static void DrawRuler2D(const tVector& origin, const tVector& size, const tVector& col, double line_width,
		double marker_spacing, double marker_h, double marker_line_width);

	static void DrawSemiCircle(const tVector& pos, double r, int slices, double min_theta, double max_theta,
		eDrawMode draw_mode = eDrawSolid);
	static void DrawCalibMarker(const tVector& pos, double r, int slices,
		const tVector& col0, const tVector& col1, eDrawMode draw_mode = eDrawSolid);

	static void DrawTexQuad(const cTextureDesc& tex, const tVector& pos, const tVector& size);

	static void CopyTexture(const cTextureDesc& src_tex, const cTextureDesc& dst_tex);
	static void CopyTexture(const cTextureDesc& src_tex, const cTextureDesc& dst_tex,
		const tVector& dst_min_coord, const tVector& dst_max_coord);
	static void CopyTexture(const cTextureDesc& src_tex);
	static void CopyTexture(const cTextureDesc& src_tex, const tVector& dst_min_coord, const tVector& dst_max_coord);

	static void ClearColor(const tVector& col);
	static void ClearDepth(double depth = 1);

	static void Translate(const tVector& trans);
	static void Scale(const tVector& scale);
	static void Rotate(const tVector& euler);
	static void Rotate(double theta, const tVector& axis);
	static void Rotate(const tQuaternion& q);

	static void SetColor(const tVector& col);
	static void SetLineWidth(double w);
	static void SetPointSize(double pt_size);

	static void MultMatrixView(const tMatrix& mat);
	static void MultMatrixProj(const tMatrix& mat);
	static void LoadMatrixView(const tMatrix& mat);
	static void LoadMatrixProj(const tMatrix& mat);
	static void LoadIdentityView();
	static void LoadIdentityProj();
	static void PushMatrixView();
	static void PushMatrixProj();
	static void PopMatrixView();
	static void PopMatrixProj();

	static void BindDefaultProg();
	static void UnbindDefaultProg();
	static void BindCopyMeshProg();
	static void UnbindCopyMeshProg();
	static void LoadShaderUniforms();

	static void Finish();

	static void BuildMeshes();

protected:
	static bool gEnableDraw;
	static tVector gColor;
	static cShader gDefaultProg;
	static cShader gCopyProg;
	static cShader gCopyMeshProg;

	static cMatrixStack gMatStackView;
	static cMatrixStack gMatStackProj;

	static std::unique_ptr<cDrawMesh> gPointMesh;
	static std::unique_ptr<cDrawMesh> gLineMesh;
	static std::unique_ptr<cDrawMesh> gQuadMesh;
	static std::unique_ptr<cDrawMesh> gBoxSolidMesh;
	static std::unique_ptr<cDrawMesh> gBoxWireMesh;
	static std::unique_ptr<cDrawMesh> gSphereMesh;
	static std::unique_ptr<cDrawMesh> gSphereWireSimpleMesh;
	static std::unique_ptr<cDrawMesh> gDiskMesh;
	static std::unique_ptr<cDrawMesh> gTriangleMesh;
	static std::unique_ptr<cDrawMesh> gCylinderMesh;
	static std::unique_ptr<cDrawMesh> gCylinderWireSimpleMesh;
	static std::unique_ptr<cDrawMesh> gConeMesh;
	static std::unique_ptr<cDrawMesh> gConeWireSimpleMesh;

	static void BuildShaders();
	static void DrawBoxSolid(const tVector& pos, const tVector& size, const tVector& tex_coord_min, const tVector& tex_coord_max);
	static void DrawBoxWire(const tVector& pos, const tVector& size);

	static void DrawCylinderSolidWire(double r, double h, eDrawMode draw_mode = eDrawSolid);
	static void DrawCylinderWireSimple(double r, double h);
	static void DrawTubeSolidWire(double r, double h, eDrawMode draw_mode = eDrawSolid);
	static void DrawConeSolidWire(double r, double h, eDrawMode draw_mode = eDrawSolid);
	static void DrawConeWireSimple(double r, double h);
};