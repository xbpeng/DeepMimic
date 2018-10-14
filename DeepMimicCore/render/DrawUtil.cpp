#include "DrawUtil.h"
#include <GL/glew.h>
#include <GL/freeglut.h>

tVector cDrawUtil::gColor = tVector::Ones();
cShader cDrawUtil::gDefaultProg = cShader();
cShader cDrawUtil::gCopyProg = cShader();
cShader cDrawUtil::gCopyMeshProg = cShader();

const int gNumSlice = 32;
const int gNumStacks = 16; // should always be even

const int gMatStackSize = 128;
cMatrixStack cDrawUtil::gMatStackView = cMatrixStack(gMatStackSize);
cMatrixStack cDrawUtil::gMatStackProj = cMatrixStack(gMatStackSize);

std::unique_ptr<cDrawMesh> cDrawUtil::gPointMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gLineMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gQuadMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gBoxSolidMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gBoxWireMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gSphereMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gSphereWireSimpleMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gDiskMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gTriangleMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gCylinderMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gCylinderWireSimpleMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gConeMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gConeWireSimpleMesh = nullptr;

bool cDrawUtil::gEnableDraw = true;

void cDrawUtil::EnableDraw(bool enable)
{
	gEnableDraw = enable;
}

bool cDrawUtil::EnableDraw()
{
	return gEnableDraw;
}

void cDrawUtil::InitOffscreenDrawContext()
{
	int argc = 0;
	glutInit(&argc, nullptr);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(1, 1);
	glutCreateWindow("DeepMimic");
}

bool cDrawUtil::CheckDrawContextInit()
{
	int state = glutGet(GLUT_INIT_STATE);
	return state == 1;
}

void cDrawUtil::InitDrawUtil()
{
	glewInit();

	const GLubyte* renderer = glGetString(GL_RENDERER); // get renderer string
	const GLubyte* version = glGetString(GL_VERSION); // version as a string
	printf("Renderer: %s\n", renderer);
	printf("OpenGL version supported %s\n", version);

	glEnable(GL_TEXTURE_2D);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glFrontFace(GL_CCW);

	BuildShaders();
	BuildMeshes();

	gMatStackView.Clear();
	gMatStackProj.Clear();
	cDrawUtil::BindDefaultProg();
}

void cDrawUtil::DrawRect(const tVector& pos, const tVector& size, eDrawMode draw_mode)
{
	tVector a = tVector(pos[0] - 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2], 0);
	tVector b = tVector(pos[0] + 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2], 0);
	tVector c = tVector(pos[0] + 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2], 0);
	tVector d = tVector(pos[0] - 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2], 0);
	DrawQuad(a, b, c, d, draw_mode);
}

void cDrawUtil::DrawBox(const tVector& pos, const tVector& size, eDrawMode draw_mode)
{
	DrawBox(pos, size, tVector::Zero(), tVector::Ones(), draw_mode);
}

void cDrawUtil::DrawBox(const tVector& pos, const tVector& size, const tVector& tex_coord_min, const tVector& tex_coord_max, eDrawMode draw_mode)
{
	if (draw_mode == eDrawWire || draw_mode == eDrawWireSimple)
	{
		DrawBoxWire(pos, size);
	}
	else if (draw_mode == eDrawSolid)
	{
		DrawBoxSolid(pos, size, tex_coord_min, tex_coord_max);
	}
	else
	{
		assert(false); // unsupported draw mode
	}
}

void cDrawUtil::DrawBoxSolid(const tVector& pos, const tVector& size, const tVector& tex_coord_min, const tVector& tex_coord_max)
{
	const int num_faces = 6;
	const int coord_len = num_faces * 6 * cMeshUtil::gCoordDim;

	const float coord_data[coord_len] = {
		static_cast<float>(tex_coord_min[0]), static_cast<float>(tex_coord_min[2]), // top
		static_cast<float>(tex_coord_max[0]), static_cast<float>(tex_coord_min[2]),
		static_cast<float>(tex_coord_max[0]), static_cast<float>(tex_coord_max[2]),
		static_cast<float>(tex_coord_max[0]), static_cast<float>(tex_coord_max[2]),
		static_cast<float>(tex_coord_min[0]), static_cast<float>(tex_coord_max[2]),
		static_cast<float>(tex_coord_min[0]), static_cast<float>(tex_coord_min[2]),

		static_cast<float>(tex_coord_max[0]), static_cast<float>(tex_coord_min[2]), // bottom
		static_cast<float>(tex_coord_max[0]), static_cast<float>(tex_coord_max[2]),
		static_cast<float>(tex_coord_min[0]), static_cast<float>(tex_coord_max[2]),
		static_cast<float>(tex_coord_min[0]), static_cast<float>(tex_coord_max[2]),
		static_cast<float>(tex_coord_min[0]), static_cast<float>(tex_coord_min[2]),
		static_cast<float>(tex_coord_max[0]), static_cast<float>(tex_coord_min[2]),

		static_cast<float>(tex_coord_min[0]), static_cast<float>(tex_coord_min[1]), // front
		static_cast<float>(tex_coord_max[0]), static_cast<float>(tex_coord_min[1]),
		static_cast<float>(tex_coord_max[0]), static_cast<float>(tex_coord_max[1]),
		static_cast<float>(tex_coord_max[0]), static_cast<float>(tex_coord_max[1]),
		static_cast<float>(tex_coord_min[0]), static_cast<float>(tex_coord_max[1]),
		static_cast<float>(tex_coord_min[0]), static_cast<float>(tex_coord_min[1]),

		static_cast<float>(tex_coord_min[0]), static_cast<float>(tex_coord_min[1]), // back
		static_cast<float>(tex_coord_max[0]), static_cast<float>(tex_coord_min[1]),
		static_cast<float>(tex_coord_max[0]), static_cast<float>(tex_coord_max[1]),
		static_cast<float>(tex_coord_max[0]), static_cast<float>(tex_coord_max[1]),
		static_cast<float>(tex_coord_min[0]), static_cast<float>(tex_coord_max[1]),
		static_cast<float>(tex_coord_min[0]), static_cast<float>(tex_coord_min[1]),

		static_cast<float>(tex_coord_max[2]), static_cast<float>(tex_coord_min[1]), // left
		static_cast<float>(tex_coord_max[2]), static_cast<float>(tex_coord_max[1]),
		static_cast<float>(tex_coord_min[2]), static_cast<float>(tex_coord_max[1]),
		static_cast<float>(tex_coord_min[2]), static_cast<float>(tex_coord_max[1]),
		static_cast<float>(tex_coord_min[2]), static_cast<float>(tex_coord_min[1]),
		static_cast<float>(tex_coord_max[2]), static_cast<float>(tex_coord_min[1]),

		static_cast<float>(tex_coord_max[2]), static_cast<float>(tex_coord_min[1]), // right
		static_cast<float>(tex_coord_max[2]), static_cast<float>(tex_coord_max[1]),
		static_cast<float>(tex_coord_min[2]), static_cast<float>(tex_coord_max[1]),
		static_cast<float>(tex_coord_min[2]), static_cast<float>(tex_coord_max[1]),
		static_cast<float>(tex_coord_min[2]), static_cast<float>(tex_coord_min[1]),
		static_cast<float>(tex_coord_max[2]), static_cast<float>(tex_coord_min[1])
	};

	tAttribInfo attr_info;
	attr_info.mAttribNumber = cMeshUtil::eAttributeCoord;
	attr_info.mAttribSize = sizeof(coord_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = cMeshUtil::gCoordDim;
	gBoxSolidMesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * coord_len, (GLubyte*)coord_data, 0, 1, &attr_info);

	PushMatrixView();
	Translate(pos);
	Scale(size);
	gBoxSolidMesh->Draw(GL_TRIANGLES);
	PopMatrixView();
}

void cDrawUtil::DrawBoxWire(const tVector& pos, const tVector& size)
{
	PushMatrixView();
	Translate(pos);
	Scale(size);
	gBoxWireMesh->Draw(GL_LINES);
	PopMatrixView();
}

void cDrawUtil::DrawTriangle(const tVector& pos, double side_len, eDrawMode draw_mode)
{
	GLenum gl_mode = (draw_mode == eDrawSolid) ? GL_TRIANGLES : GL_LINE_LOOP;
	PushMatrixView();
	Translate(pos);
	Scale(tVector(side_len, side_len, side_len, 1));
	gTriangleMesh->Draw(gl_mode);
	PopMatrixView();
}

void cDrawUtil::DrawQuad(const tVector& a, const tVector& b, const tVector& c, const tVector& d, eDrawMode draw_mode)
{
	DrawQuad(a, b, c, d, tVector(0, 0, 0, 0), tVector(1, 0, 0, 0),
		tVector(1, 1, 0, 0), tVector(0, 1, 0, 0), draw_mode);
}

void cDrawUtil::DrawQuad(const tVector& a, const tVector& b, const tVector& c, const tVector& d,
	const tVector& coord_a, const tVector& coord_b, const tVector& coord_c, const tVector& coord_d,
	eDrawMode draw_mode)
{
	const int num_verts = 4;
	const int pos_len = num_verts * cMeshUtil::gPosDim;
	const int norm_len = num_verts * cMeshUtil::gNormDim;
	const int coord_len = num_verts * cMeshUtil::gCoordDim;

	GLenum gl_mode = (draw_mode == eDrawSolid) ? GL_TRIANGLE_FAN : GL_LINE_LOOP;

	tVector normal = (b - a).cross3(d - a);
	double normal_len = normal.norm();
	if (normal_len == 0)
	{
		normal = tVector(0, 0, 1, 0);
	}
	else
	{
		normal = (normal / normal_len);
	}

	const float pos_data[pos_len] =
	{
		static_cast<float>(a[0]), static_cast<float>(a[1]), static_cast<float>(a[2]),
		static_cast<float>(b[0]), static_cast<float>(b[1]), static_cast<float>(b[2]),
		static_cast<float>(c[0]), static_cast<float>(c[1]), static_cast<float>(c[2]),
		static_cast<float>(d[0]), static_cast<float>(d[1]), static_cast<float>(d[2])
	};

	const float norm_data[norm_len] =
	{
		static_cast<float>(normal[0]), static_cast<float>(normal[1]), static_cast<float>(normal[2]),
		static_cast<float>(normal[0]), static_cast<float>(normal[1]), static_cast<float>(normal[2]),
		static_cast<float>(normal[0]), static_cast<float>(normal[1]), static_cast<float>(normal[2]),
		static_cast<float>(normal[0]), static_cast<float>(normal[1]), static_cast<float>(normal[2])
	};

	const float coord_data[coord_len] =
	{
		static_cast<float>(coord_a[0]), static_cast<float>(coord_a[1]),
		static_cast<float>(coord_b[0]), static_cast<float>(coord_b[1]),
		static_cast<float>(coord_c[0]), static_cast<float>(coord_c[1]),
		static_cast<float>(coord_d[0]), static_cast<float>(coord_d[1])
	};

	tAttribInfo attr_info;
	attr_info.mAttribNumber = cMeshUtil::eAttributePosition;
	attr_info.mAttribSize = sizeof(pos_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = cMeshUtil::gPosDim;
	gQuadMesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * pos_len, (GLubyte*)pos_data, 0, 1, &attr_info);

	attr_info.mAttribNumber = cMeshUtil::eAttributeNormal;
	attr_info.mAttribSize = sizeof(norm_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = cMeshUtil::gNormDim;
	gQuadMesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * norm_len, (GLubyte*)norm_data, 0, 1, &attr_info);

	attr_info.mAttribNumber = cMeshUtil::eAttributeCoord;
	attr_info.mAttribSize = sizeof(norm_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = cMeshUtil::gCoordDim;
	gQuadMesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * coord_len, (GLubyte*)coord_data, 0, 1, &attr_info);

	gQuadMesh->Draw(gl_mode);
}

void cDrawUtil::DrawDisk(const tVector& pos, double r, eDrawMode draw_mode)
{
	DrawDisk(pos, tVector(r, r, r, 1), draw_mode);
}

void cDrawUtil::DrawDisk(const tVector& pos, const tVector& r, eDrawMode draw_mode)
{
	cDrawUtil::PushMatrixView();
	cDrawUtil::Translate(pos);
	DrawDisk(r, draw_mode);
	cDrawUtil::PopMatrixView();
}

void cDrawUtil::DrawDisk(double r, eDrawMode draw_mode)
{
	cDrawUtil::DrawDisk(tVector(r, r, r, 1), draw_mode);
}

void cDrawUtil::DrawDisk(const tVector& r, eDrawMode draw_mode)
{
	cDrawUtil::PushMatrixView();
	cDrawUtil::Scale(r);

	if (draw_mode == eDrawWireSimple || draw_mode == eDrawWire)
	{
		gDiskMesh->Draw(GL_LINE_STRIP, 1);
	}
	else if (draw_mode == eDrawSolid)
	{
		gDiskMesh->Draw(GL_TRIANGLE_FAN);
	}
	else
	{
		assert(false); //unsupported draw mode
	}

	cDrawUtil::PopMatrixView();
}

void cDrawUtil::DrawPoint(const tVector& pt)
{
	cDrawUtil::PushMatrixView();
	cDrawUtil::Translate(pt);
	gPointMesh->Draw(GL_POINTS);
	cDrawUtil::PopMatrixView();
}

void cDrawUtil::DrawLine(const tVector& a, const tVector& b)
{
	const int num_verts = 2;
	const int pos_len = num_verts * cMeshUtil::gPosDim;
	const float pos_data[pos_len] =
	{
		static_cast<float>(a[0]), static_cast<float>(a[1]), static_cast<float>(a[2]),
		static_cast<float>(b[0]), static_cast<float>(b[1]), static_cast<float>(b[2])
	};

	tAttribInfo attr_info;
	attr_info.mAttribNumber = cMeshUtil::eAttributePosition;
	attr_info.mAttribSize = sizeof(pos_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = cMeshUtil::gPosDim;
	gLineMesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * pos_len, (GLubyte*)pos_data, 0, 1, &attr_info);

	gLineMesh->Draw(GL_LINES);
}

void cDrawUtil::DrawLineStrip(const tVectorArr& pts)
{
	int num_pts = static_cast<int>(pts.size());
	glBegin(GL_LINE_STRIP);
	for (int i = 0; i < num_pts - 1; ++i)
	{
		const tVector& a = pts[i];
		const tVector& b = pts[i + 1];
		glVertex3d(a[0], a[1], a[2]);
		glVertex3d(b[0], b[1], b[2]);
	}
	glEnd();
}

void cDrawUtil::DrawStrip(const tVector& a, const tVector& b, double width, eDrawMode draw_mode)
{
	tVector delta = b - a;
	tVector orthogonal = tVector(-delta[1], delta[0], 0, 0);
	orthogonal.normalize();

	tVector v0 = a - width * 0.5 * orthogonal;
	tVector v1 = b - width * 0.5 * orthogonal;
	tVector v2 = b + width * 0.5 * orthogonal;
	tVector v3 = a + width * 0.5 * orthogonal;

	DrawQuad(v0, v1, v2, v3, draw_mode);
}

void cDrawUtil::DrawCross(const tVector& pos, double size)
{
	DrawLine(tVector(pos[0] - 0.5 * size, pos[1], pos[2], 0),
		tVector(pos[0] + 0.5 * size, pos[1], pos[2], 0));
	DrawLine(tVector(pos[0], pos[1] - 0.5 * size, pos[2], 0),
		tVector(pos[0], pos[1] + 0.5 * size, pos[2], 0));
	DrawLine(tVector(pos[0], pos[1], pos[2] - 0.5 * size, 0),
		tVector(pos[0], pos[1], pos[2] + 0.5 * size, 0));
}

void cDrawUtil::DrawSphere(double r, eDrawMode draw_mode)
{
	cDrawUtil::PushMatrixView();
	cDrawUtil::Scale(tVector(r, r, r, 1));

	if (draw_mode == eDrawSolid)
	{
		gSphereMesh->Draw(GL_TRIANGLES);
	}
	else if (draw_mode == eDrawWire)
	{
		gSphereMesh->Draw(GL_LINES);
	}
	else if (draw_mode == eDrawWireSimple)
	{
		gSphereWireSimpleMesh->Draw(GL_LINES);
	}
	else
	{
		assert(false); // unsupported draw mode
	}

	cDrawUtil::PopMatrixView();
}

void cDrawUtil::DrawHemisphere(double r, eDrawMode draw_mode)
{
	cDrawUtil::PushMatrixView();
	cDrawUtil::Scale(tVector(r, r, r, 1));
	int elem = gSphereMesh->GetNumVerts() / 2;

	if (draw_mode == eDrawSolid)
	{
		gSphereMesh->Draw(GL_TRIANGLES, 0, elem);
	}
	else if (draw_mode == eDrawWire)
	{
		gSphereMesh->Draw(GL_LINES, 0, elem);
		cDrawUtil::Rotate(tVector(-M_PI / 2, 0, 0, 0));
		gDiskMesh->Draw(GL_LINE_STRIP);
	}
	else if (draw_mode == eDrawWireSimple)
	{
		gSphereWireSimpleMesh->Draw(GL_LINES, 0, ((gNumStacks / 2) * 4 + gNumSlice) * 2);
	}
	else
	{
		assert(false); // unsupported draw mode
	}

	cDrawUtil::PopMatrixView();
}

void cDrawUtil::DrawCylinder(double r, double h, eDrawMode draw_mode)
{
	if (draw_mode == eDrawSolid || draw_mode == eDrawWire)
	{
		DrawCylinderSolidWire(r, h, draw_mode);
	}
	else if (draw_mode == eDrawWireSimple)
	{
		DrawCylinderWireSimple(r, h);
	}
	else
	{
		assert(false); // unsupported draw mode
	}
}

void cDrawUtil::DrawCone(double r, double h, eDrawMode draw_mode)
{
	if (draw_mode == eDrawSolid || draw_mode == eDrawWire)
	{
		DrawConeSolidWire(r, h, draw_mode);
	}
	else if (draw_mode == eDrawWireSimple)
	{
		DrawConeWireSimple(r, h);
	}
	else
	{
		assert(false); // unsupported draw mode
	}
}

void cDrawUtil::DrawTube(double r, double h, eDrawMode draw_mode)
{
	if (draw_mode == eDrawSolid || draw_mode == eDrawWire)
	{
		DrawTubeSolidWire(r, h, draw_mode);
	}
	else if (draw_mode == eDrawWireSimple)
	{
		DrawCylinderWireSimple(r, h);
	}
	else
	{
		assert(false); // unsupported draw mode
	}
}

void cDrawUtil::DrawPlane(const tVector& coeffs, double size, eDrawMode draw_mode)
{
	const Eigen::Vector3d ref = Eigen::Vector3d(0, 0, 1);
	Eigen::Vector3d n = Eigen::Vector3d(coeffs[0], coeffs[1], coeffs[2]);
	double c = coeffs[3];

	Eigen::Vector3d axis = ref.cross(n);
	double axis_len = axis.norm();
	double theta = 0;
	if (axis_len != 0)
	{
		axis /= axis_len;
		theta = std::acos(ref.dot(n));
	}

	Eigen::Vector3d offset = c * n;

	cDrawUtil::PushMatrixView();
	cDrawUtil::Translate(tVector(offset[0], offset[1], offset[2], 0));
	if (theta != 0)
	{
		cDrawUtil::Rotate(theta, tVector(axis[0], axis[1], axis[2], 0));
	}
	DrawRect(tVector::Zero(), tVector(size, size, 0, 0), draw_mode);
	cDrawUtil::PopMatrixView();
}

void cDrawUtil::DrawCapsule(double r, double h, eDrawMode draw_mode)
{
	PushMatrixView();

	DrawTube(r, h, draw_mode);

	Translate(tVector(0, 0.5 * h, 0, 0));
	DrawHemisphere(r, draw_mode);

	Translate(tVector(0, -h, 0, 0));
	Rotate(tVector(0, 0, M_PI, 0));
	DrawHemisphere(r, draw_mode);

	PopMatrixView();
}

void cDrawUtil::DrawArrow2D(const tVector& start, const tVector& end, double head_size)
{
	GLboolean prev_enable;
	glGetBooleanv(GL_CULL_FACE, &prev_enable);
	glDisable(GL_CULL_FACE);

	tVector dir = tVector(0, 1, 0, 0);
	double dir_len = 0;
	if (start != end)
	{
		dir = end - start;
		dir_len = dir.norm();
		dir /= dir_len;
	}

	dir[3] = 0;
	tVector axis = tVector(0, 0, 1, 0);
	tVector tangent = axis.cross3(dir);
	tangent.normalize();

	const double width = head_size * 0.1854;
	tVector body_end = end - dir * head_size;

	tVector a = start - width * tangent;
	tVector b = body_end - width * tangent;
	tVector c = body_end + width * tangent;
	tVector d = start + width * tangent;
	DrawQuad(a, b, c, d);

	tVector e0 = body_end - tangent * head_size * 0.5f;
	tVector e1 = body_end + tangent * head_size * 0.5f;
	DrawQuad(end, e1, e0, end);

	if (prev_enable)
	{
		glEnable(GL_CULL_FACE);
	}
}

void cDrawUtil::DrawArrow3D(const tVector& start, const tVector& end, double head_size)
{
	tVector dir = tVector(0, 1, 0, 0);
	double dir_len = 0;
	if (start != end)
	{
		dir = end - start;
		dir_len = dir.norm();
		dir /= dir_len;
	}

	tQuaternion rot = cMathUtil::VecDiffQuat(tVector(0, 1, 0, 0), dir);
	tMatrix rot_mat = cMathUtil::RotateMat(rot);

	double body_len = dir_len - head_size;
	double body_radius = head_size * 0.1854;
	double head_len = head_size;
	double head_radius = 0.5 * head_size;

	cDrawUtil::PushMatrixView();

	cDrawUtil::Translate(start);
	cDrawUtil::MultMatrixView(rot_mat);
	cDrawUtil::Translate(tVector(0, body_len, 0, 0));
	cDrawUtil::DrawCone(head_radius, head_len, cDrawUtil::eDrawSolid);
	cDrawUtil::Translate(tVector(0, -0.5 * body_len, 0, 0));
	cDrawUtil::DrawCylinder(body_radius, body_len, cDrawUtil::eDrawSolid);

	cDrawUtil::PopMatrixView();
}

void cDrawUtil::DrawGrid2D(const tVector& origin, const tVector& size, double spacing, double line_width)
{
	double w = size[0];
	double h = size[1];

	double min_x = origin(0) - w;
	double min_y = origin(1) - h;
	double max_x = origin(0) + w;
	double max_y = origin(1) + h;

	const double offset_z = origin[2];

	cDrawUtil::SetLineWidth(line_width);
	cDrawUtil::SetColor(tVector(188 / 255.f, 219 / 255.f, 242 / 255.f, 1.f));

	for (double x = min_x - std::fmod(min_x, spacing); x < max_x; x += spacing)
	{
		tVector a = tVector(x, min_y, offset_z, offset_z);
		tVector b = tVector(x, max_y, offset_z, offset_z);
		cDrawUtil::DrawLine(a, b);
	}

	for (double y = min_y - std::fmod(min_y, spacing); y < max_y; y += spacing)
	{
		tVector a = tVector(min_x, y, offset_z, offset_z);
		tVector b = tVector(max_x, y, offset_z, offset_z);
		cDrawUtil::DrawLine(a, b);
	}
}

void cDrawUtil::DrawRuler2D(const tVector& origin, const tVector& size,
	const tVector& col, double line_width, double marker_spacing,
	double marker_h, double marker_line_width)
{
	double w = size[0];
	double h = size[1];

	double min_x = origin(0) - w * 0.5;
	double max_x = origin(0) + w * 0.5;
	double max_y = origin(1) + h * 0.5;

	if (line_width > 0)
	{
		glTexCoord2d(0, 0);
		cDrawUtil::SetLineWidth(line_width);
		cDrawUtil::SetColor(col);
		cDrawUtil::DrawRect(origin, size, cDrawUtil::eDrawSolid);
		cDrawUtil::SetColor(tVector(0, 0, 0, 1));
		cDrawUtil::DrawLine(tVector(min_x, max_y, 0, 0), tVector(max_x, max_y, 0, 0));
	}

	// draw markers
	if (marker_line_width > 0)
	{
		cDrawUtil::SetColor(tVector(0.f, 0.f, 0.f, 1.f));
		cDrawUtil::SetLineWidth(marker_line_width);
		for (double x = min_x - std::fmod(min_x, marker_spacing); x < max_x; x += marker_spacing)
		{
			tVector a = tVector(x, max_y + marker_h * 0.5f, 0, 0);
			tVector b = tVector(x, max_y - marker_h * 0.5f, 0, 0);
			cDrawUtil::DrawLine(a, b);
		}
	}
}

void cDrawUtil::DrawSemiCircle(const tVector& pos, double r, int slices,
	double min_theta, double max_theta, eDrawMode draw_mode)
{
	double d_theta = (max_theta - min_theta) / (slices - 1);

	GLenum gl_mode = (draw_mode == eDrawSolid) ? GL_TRIANGLE_FAN : GL_LINE_LOOP;
	glBegin(gl_mode);

	glTexCoord2d(0, 0);
	glNormal3d(0, 0, 1);
	glVertex3d(pos[0], pos[1], pos[2]);
	for (int i = 0; i < slices; ++i)
	{
		double theta0 = i * d_theta + min_theta;
		double theta1 = i * d_theta + min_theta;

		double x0 = r * std::cos(theta0) + pos[0];
		double y0 = r * std::sin(theta0) + pos[1];
		double x1 = r * std::cos(theta1) + pos[0];
		double y1 = r * std::sin(theta1) + pos[1];;

		glVertex3d(x0, y0, pos[2]);
		glVertex3d(x1, y1, pos[2]);
	}
	glEnd();
}

void cDrawUtil::DrawCalibMarker(const tVector& pos, double r, int slices,
	const tVector& col0, const tVector& col1,
	eDrawMode draw_mode)
{
	SetColor(col0);
	DrawSemiCircle(pos, r, slices, 0, M_PI * 0.5, draw_mode);
	DrawSemiCircle(pos, r, slices, M_PI, M_PI * 1.5, draw_mode);

	SetColor(col1);
	DrawSemiCircle(pos, r, slices, M_PI * 0.5, M_PI, draw_mode);
	DrawSemiCircle(pos, r, slices, M_PI * 1.5, M_PI * 2, draw_mode);
}

void cDrawUtil::DrawTexQuad(const cTextureDesc& tex, const tVector& pos, const tVector& size)
{
	tex.BindTex(GL_TEXTURE0);
	DrawQuad(tVector(pos[0] - 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2], 0),
		tVector(pos[0] + 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2], 0),
		tVector(pos[0] + 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2], 0),
		tVector(pos[0] - 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2], 0),
		tVector(0, 0, 0, 0), tVector(1, 0, 0, 0), tVector(1, 1, 0, 0), tVector(0, 1, 0, 0),
		eDrawSolid);
	tex.UnbindTex(GL_TEXTURE0);
}

void cDrawUtil::CopyTexture(const cTextureDesc& src_tex, const cTextureDesc& dst_tex)
{
	CopyTexture(src_tex, tVector(-1, -1, 0, 0), tVector(1, 1, 0, 0));
}

void cDrawUtil::CopyTexture(const cTextureDesc& src_tex, const cTextureDesc& dst_tex,
	const tVector& dst_min_coord, const tVector& dst_max_coord)
{
	dst_tex.BindBuffer();
	CopyTexture(src_tex, dst_min_coord, dst_max_coord);
	dst_tex.UnbindBuffer();
}

void cDrawUtil::CopyTexture(const cTextureDesc& src_tex)
{
	CopyTexture(src_tex, tVector(-1, -1, 0, 0), tVector(1, 1, 0, 0));
}

void cDrawUtil::CopyTexture(const cTextureDesc& src_tex, const tVector& dst_min_coord, const tVector& dst_max_coord)
{
	cDrawUtil::PushMatrixProj();
	cDrawUtil::LoadIdentityProj();

	cDrawUtil::PushMatrixView();
	cDrawUtil::LoadIdentityView();

	GLint depth_test_param;
	glGetIntegerv(GL_DEPTH_TEST, &depth_test_param);
	glDisable(GL_DEPTH_TEST);

	GLint blend_param;
	glGetIntegerv(GL_BLEND, &blend_param);
	glDisable(GL_BLEND);

	SetColor(tVector::Ones());
	gCopyProg.Bind();

	tVector quad_pos = 0.5 * (dst_max_coord + dst_min_coord);
	tVector quad_size = dst_max_coord - dst_min_coord;
	DrawTexQuad(src_tex, quad_pos, quad_size);

	gCopyProg.Unbind();

	if (depth_test_param == 1)
	{
		glEnable(GL_DEPTH_TEST);
	}

	if (blend_param == 1)
	{
		glEnable(GL_BLEND);
	}

	cDrawUtil::PopMatrixView();
	cDrawUtil::PopMatrixProj();
}

void cDrawUtil::ClearColor(const tVector& col)
{
	glClearColor(static_cast<float>(col[0]), static_cast<float>(col[1]),
		static_cast<float>(col[2]), static_cast<float>(col[3]));
	glClear(GL_COLOR_BUFFER_BIT);
}

void cDrawUtil::ClearDepth(double depth /* = 1*/)
{
	glClearDepth(depth);
	glClear(GL_DEPTH_BUFFER_BIT);
}


void cDrawUtil::MultMatrixView(const tMatrix& mat)
{
	gMatStackView.MultMatrix(mat);
}

void cDrawUtil::MultMatrixProj(const tMatrix& mat)
{
	gMatStackProj.MultMatrix(mat);
}

void cDrawUtil::LoadMatrixView(const tMatrix& mat)
{
	gMatStackView.SetMatrix(mat);
}

void cDrawUtil::LoadMatrixProj(const tMatrix& mat)
{
	gMatStackProj.SetMatrix(mat);
}

void cDrawUtil::LoadIdentityView()
{
	gMatStackView.SetIdentity();
}

void cDrawUtil::LoadIdentityProj()
{
	gMatStackProj.SetIdentity();
}

void cDrawUtil::PushMatrixView()
{
	gMatStackView.Push();
}

void cDrawUtil::PushMatrixProj()
{
	gMatStackProj.Push();
}

void cDrawUtil::PopMatrixView()
{
	gMatStackView.Pop();
}

void cDrawUtil::PopMatrixProj()
{
	gMatStackProj.Pop();
}

void cDrawUtil::BindDefaultProg()
{
	gDefaultProg.Bind();
}

void cDrawUtil::UnbindDefaultProg()
{
	gDefaultProg.Unbind();
}

void cDrawUtil::BindCopyMeshProg()
{
	gCopyMeshProg.Bind();
}

void cDrawUtil::UnbindCopyMeshProg()
{
	gCopyMeshProg.Unbind();
}

void cDrawUtil::LoadShaderUniforms()
{
	const cShader* curr_shader = cShader::GetBoundShader();
	if (curr_shader != nullptr)
	{
		if (curr_shader->HasUniform(cShader::eUniformModelViewMatrix))
		{
			tMatrix view_mat = gMatStackView.GetMatrix();
			curr_shader->SetUniformMat(cShader::eUniformModelViewMatrix, view_mat);
		}

		if (curr_shader->HasUniform(cShader::eUniformProjectionMatrix))
		{
			tMatrix proj_mat = gMatStackProj.GetMatrix();
			curr_shader->SetUniformMat(cShader::eUniformProjectionMatrix, proj_mat);
		}

		if (curr_shader->HasUniform(cShader::eUniformColor))
		{
			curr_shader->SetUniform4(cShader::eUniformColor, gColor);
		}
	}
}

void cDrawUtil::Finish()
{
	glFinish();
}

void cDrawUtil::BuildMeshes()
{
	cMeshUtil::BuildPointMesh(gPointMesh);
	cMeshUtil::BuildLineMesh(gLineMesh);
	cMeshUtil::BuildQuadMesh(gQuadMesh);
	cMeshUtil::BuildBoxSolidMesh(gBoxSolidMesh);
	cMeshUtil::BuildBoxWireMesh(gBoxWireMesh);
	cMeshUtil::BuildSphereMesh(gNumStacks, gNumSlice, gSphereMesh);
	cMeshUtil::BuildSphereWireSimpleMesh(gNumStacks, gNumSlice, gSphereWireSimpleMesh);
	cMeshUtil::BuildDiskMesh(gNumSlice, gDiskMesh);
	cMeshUtil::BuildTriangleMesh(gTriangleMesh);
	cMeshUtil::BuildCylinder(gNumSlice, gCylinderMesh);
	cMeshUtil::BuildCylinderWireSimple(gNumSlice, gCylinderWireSimpleMesh);
	cMeshUtil::BuildCone(gNumSlice, gConeMesh);
	cMeshUtil::BuildConeWireSimple(gNumSlice, gConeWireSimpleMesh);
}

void cDrawUtil::Translate(const tVector& trans)
{
	gMatStackView.Translate(trans);
}

void cDrawUtil::Scale(const tVector& scale)
{
	gMatStackView.Scale(scale);
}

void cDrawUtil::Rotate(const tVector& euler)
{
	double theta;
	tVector axis;
	cMathUtil::EulerToAxisAngle(euler, axis, theta);
	Rotate(theta, axis);
}

void cDrawUtil::Rotate(double theta, const tVector& axis)
{
	gMatStackView.Rotate(theta, axis);
}

void cDrawUtil::Rotate(const tQuaternion& q)
{
	double theta;
	tVector axis;
	cMathUtil::QuaternionToAxisAngle(q, axis, theta);
	Rotate(theta, axis);
}

void cDrawUtil::SetColor(const tVector& col)
{
	gColor = col;
}

void cDrawUtil::SetLineWidth(double w)
{
	glLineWidth(static_cast<float>(w));
}

void cDrawUtil::SetPointSize(double pt_size)
{
	glPointSize(static_cast<float>(pt_size));
}

void cDrawUtil::BuildShaders()
{
	{
		gDefaultProg.BuildShader("data/shaders/Mesh_VS.glsl", "data/shaders/VertColor_PS.glsl");
	}

	{
		gCopyProg.BuildShader("data/shaders/FullScreenQuad_VS.glsl", "data/shaders/DownSample_PS.glsl");
	}

	{
		gCopyMeshProg.BuildShader("data/shaders/Mesh_VS.glsl", "data/shaders/DownSample_PS.glsl");
	}
}

void cDrawUtil::DrawCylinderSolidWire(double r, double h, eDrawMode draw_mode)
{
	GLenum gl_mode = (draw_mode == eDrawSolid) ? GL_TRIANGLES : GL_LINES;
	PushMatrixView();
	Scale(tVector(r, h, r, 0));
	gCylinderMesh->Draw(gl_mode);
	PopMatrixView();
}

void cDrawUtil::DrawCylinderWireSimple(double r, double h)
{
	PushMatrixView();
	Scale(tVector(r, h, r, 0));
	gCylinderWireSimpleMesh->Draw(GL_LINES);
	PopMatrixView();
}

void cDrawUtil::DrawTubeSolidWire(double r, double h, eDrawMode draw_mode)
{
	PushMatrixView();
	Scale(tVector(r, h, r, 0));
	if (draw_mode == eDrawSolid)
	{
		gCylinderMesh->Draw(GL_TRIANGLES, 0, gNumSlice * 6);
	}
	else
	{
		gCylinderMesh->Draw(GL_LINES);
	}
	PopMatrixView();
}

void cDrawUtil::DrawConeSolidWire(double r, double h, eDrawMode draw_mode)
{
	GLenum gl_mode = (draw_mode == eDrawSolid) ? GL_TRIANGLES : GL_LINES;
	PushMatrixView();
	Scale(tVector(r, h, r, 0));
	gConeMesh->Draw(gl_mode);
	PopMatrixView();
}

void cDrawUtil::DrawConeWireSimple(double r, double h)
{
	PushMatrixView();
	Scale(tVector(r, h, r, 0));
	gConeWireSimpleMesh->Draw(GL_LINES);
	PopMatrixView();
}