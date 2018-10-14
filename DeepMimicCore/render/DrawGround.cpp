#include "DrawGround.h"
#include "DrawUtil.h"
#include "sim/GroundPlane.h"

void cDrawGround::BuildMesh(const cGround* ground, cDrawMesh* out_mesh)
{
	cGround::eClass ground_class = ground->GetGroundClass();
	switch (ground_class)
	{
	case cGround::eClassPlane:
		BuildMeshPlane(ground, out_mesh);
		break;
	default:
		assert(false); // unsupported ground type
		break;
	}
}

void cDrawGround::BuildMeshPlane(const cGround* ground, cDrawMesh* out_mesh)
{
	const cGroundPlane* ground_plane = reinterpret_cast<const cGroundPlane*>(ground);
	const tVector& center = ground_plane->GetPrevCenter();

	const tVector tex_size = tVector(0.5, 0.5, 0, 0);
	const int num_faces = 1;
	const int verts_per_face = 6;
	const int vert_size = 3;
	const int norm_size = 3;
	const int coord_size = 2;
	const double w = 200;

	tVector size = tVector(w, 0, w, 0);
	tVector ground_origin = ground->GetPos();

	tVector a = tVector(center[0] - 0.5 * size[0], ground_origin[1], center[2] - 0.5 * size[2], 0);
	tVector b = tVector(center[0] - 0.5 * size[0], ground_origin[1], center[2] + 0.5 * size[2], 0);
	tVector c = tVector(center[0] + 0.5 * size[0], ground_origin[1], center[2] + 0.5 * size[2], 0);
	tVector d = tVector(center[0] + 0.5 * size[0], ground_origin[1], center[2] - 0.5 * size[2], 0);

	tVector min_coord = a - ground_origin;
	tVector max_coord = c - ground_origin;
	min_coord[0] /= tex_size[0];
	min_coord[1] = min_coord[2] / tex_size[1];
	max_coord[0] /= tex_size[0];
	max_coord[1] = max_coord[2] / tex_size[1];

	tVector coord_a = tVector(min_coord[0], min_coord[1], 0, 0);
	tVector coord_b = tVector(min_coord[0], max_coord[1], 0, 0);
	tVector coord_c = tVector(max_coord[0], max_coord[1], 0, 0);
	tVector coord_d = tVector(max_coord[0], min_coord[1], 0, 0);
	
	const int vert_len = num_faces * verts_per_face * vert_size;
	const int norm_len = num_faces * verts_per_face * norm_size;
	const int coord_len = num_faces * verts_per_face * coord_size;
	const int idx_len = num_faces * verts_per_face;

	float vert_data[vert_len] =
	{
		static_cast<float>(a[0]), static_cast<float>(a[1]), static_cast<float>(a[2]),
		static_cast<float>(b[0]), static_cast<float>(b[1]), static_cast<float>(b[2]),
		static_cast<float>(c[0]), static_cast<float>(c[1]), static_cast<float>(c[2]),
		static_cast<float>(c[0]), static_cast<float>(c[1]), static_cast<float>(c[2]),
		static_cast<float>(d[0]), static_cast<float>(d[1]), static_cast<float>(d[2]),
		static_cast<float>(a[0]), static_cast<float>(a[1]), static_cast<float>(a[2])
	};

	float norm_data[norm_len] = 
	{
		0.f, 1.f, 0.f,
		0.f, 1.f, 0.f,
		0.f, 1.f, 0.f,
		0.f, 1.f, 0.f,
		0.f, 1.f, 0.f,
		0.f, 1.f, 0.f
	};

	float coord_data[coord_len] =
	{
		static_cast<float>(coord_a[0]), static_cast<float>(coord_a[1]),
		static_cast<float>(coord_b[0]), static_cast<float>(coord_b[1]),
		static_cast<float>(coord_c[0]), static_cast<float>(coord_c[1]),
		static_cast<float>(coord_c[0]), static_cast<float>(coord_c[1]),
		static_cast<float>(coord_d[0]), static_cast<float>(coord_d[1]),
		static_cast<float>(coord_a[0]), static_cast<float>(coord_a[1])
	};

	int idx_data[idx_len];
	for (int i = 0; i < idx_len; ++i)
	{
		idx_data[i] = i;
	}

	cMeshUtil::BuildDrawMesh(vert_data, vert_len, norm_data, norm_len, coord_data, coord_len, idx_data, idx_len, out_mesh);
}