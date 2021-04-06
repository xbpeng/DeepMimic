#include "MeshUtil.h"
#include "util/MathUtil.h"
#include "render/OBJLoader.h"

cMeshUtil::tVertex::tVertex()
{
	mPosition.setZero();
	mNormal = tVector(0, 1, 0, 0);
	mCoord.setZero();
}

cMeshUtil::tRayTestResult::tRayTestResult()
{
	mFace = gInvalidIdx;
	mDist = 0;
	mIntersect.setZero();
}

void cMeshUtil::BuildDrawMesh(const float* pos_data, int pos_size, const int* idx_data, int idx_size,
								cDrawMesh* out_mesh)
{
	const int face_dim = 3;
	std::vector<float> norm_data;
	std::vector<float> coord_data;

	int num_verts = pos_size / gPosDim;
	int num_faces = idx_size / face_dim;

	norm_data.resize(num_verts * gNormDim);
	for (int v = 0; v < num_verts; ++v)
	{
		for (int i = 0; i < gNormDim; ++i)
		{
			norm_data[v * gNormDim + i] = 0;
		}

		coord_data.push_back(0);
		coord_data.push_back(0);
	}

	for (int f = 0; f < num_faces; ++f)
	{
		int i0 = idx_data[f * face_dim];
		int i1 = idx_data[f * face_dim + 1];
		int i2 = idx_data[f * face_dim + 2];

		tVector v0 = tVector(pos_data[i0 * gPosDim], pos_data[i0 * gPosDim + 1], pos_data[i0 * gPosDim + 2], 0);
		tVector v1 = tVector(pos_data[i1 * gPosDim], pos_data[i1 * gPosDim + 1], pos_data[i1 * gPosDim + 2], 0);
		tVector v2 = tVector(pos_data[i2 * gPosDim], pos_data[i2 * gPosDim + 1], pos_data[i2 * gPosDim + 2], 0);

		tVector normal = (v1 - v0).cross3(v2 - v0);
		normal.normalize();
		for (int i = 0; i < gNormDim; ++i)
		{
			norm_data[i0 * gPosDim + i] += static_cast<float>(normal[i]);
			norm_data[i1 * gPosDim + i] += static_cast<float>(normal[i]);
			norm_data[i2 * gPosDim + i] += static_cast<float>(normal[i]);
		}
	}

	for (int v = 0; v < num_verts; ++v)
	{
		float len = 0;
		for (int i = 0; i < gNormDim; ++i)
		{
			float n_val = norm_data[v * gNormDim + i];
			len += n_val * n_val;
		}

		len = std::sqrt(len);
		for (int i = 0; i < gNormDim; ++i)
		{
			norm_data[v * gNormDim + i] /= len;
		}
	}

	BuildDrawMesh(pos_data, pos_size, norm_data.data(), static_cast<int>(norm_data.size()),
					coord_data.data(), static_cast<int>(coord_data.size()), idx_data, idx_size,
					out_mesh);
}

void cMeshUtil::BuildDrawMesh(const float* pos_data, int pos_size, const float* norm_data, int norm_size,
								const float* coord_data, int coord_size, const int* idx_data, int idx_size,
								cDrawMesh* out_mesh)
{
	out_mesh->Init(1);

	tAttribInfo attr_info;
	attr_info.mAttribNumber = eAttributePosition;
	attr_info.mAttribSize = sizeof(pos_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = gPosDim;
	out_mesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * pos_size, (GLubyte*) pos_data, 0, 1, &attr_info);

	attr_info.mAttribNumber = eAttributeNormal;
	attr_info.mAttribSize = sizeof(norm_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = gNormDim;
	out_mesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * norm_size, (GLubyte*) norm_data, 0, 1, &attr_info);
	
	attr_info.mAttribNumber = eAttributeCoord;
	attr_info.mAttribSize = sizeof(coord_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = gCoordDim;
	out_mesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * coord_size, (GLubyte*) coord_data, 0, 1, &attr_info);

	out_mesh->LoadIBuffer(idx_size, sizeof(int), (int*) idx_data);
}

cMeshUtil::tVertex cMeshUtil::GetVertex(int v, const cDrawMesh& mesh)
{
	const float* pos_data = mesh.GetData(eAttributePosition);
	const float* norm_data = mesh.GetData(eAttributeNormal);
	const float* coord_data = mesh.GetData(eAttributeCoord);

	tVertex vert;
	vert.mPosition = tVector(pos_data[v * gPosDim], pos_data[v * gPosDim + 1], pos_data[v * gPosDim + 2], 0);
	vert.mNormal = tVector(norm_data[v * gNormDim], norm_data[v * gNormDim + 1], norm_data[v * gNormDim + 2], 0);
	vert.mCoord = Eigen::Vector2d(coord_data[v * gCoordDim], coord_data[v * gCoordDim + 1]);

	return vert;
}

cMeshUtil::tFace cMeshUtil::GetFace(int f, const cDrawMesh& mesh)
{
	const int* idx_data = mesh.GetIdxData();
	return tFace(idx_data[f * gVertsPerTriangle], idx_data[f * gVertsPerTriangle + 1], idx_data[f * gVertsPerTriangle + 2]);
}

void cMeshUtil::ExpandFaces(const cDrawMesh& mesh, std::shared_ptr<cDrawMesh>& out_mesh)
{
	int num_faces = mesh.GetNumFaces();

	std::vector<float> new_pos_data(num_faces * gVertsPerTriangle * gPosDim);
	std::vector<float> new_norm_data(num_faces * gVertsPerTriangle * gNormDim);
	std::vector<float> new_coord_data(num_faces * gVertsPerTriangle * gCoordDim);
	std::vector<int> new_idx_data(num_faces * gVertsPerTriangle);

	for (int f = 0; f < num_faces; ++f)
	{
		tFace face = GetFace(f, mesh);
		tVertex v0 = GetVertex(face[0], mesh);
		tVertex v1 = GetVertex(face[1], mesh);
		tVertex v2 = GetVertex(face[2], mesh);

		tVector normal = (v1.mPosition - v0.mPosition).cross3(v2.mPosition - v0.mPosition);
		normal.normalize();

		int i0 = f * gVertsPerTriangle;
		int i1 = f * gVertsPerTriangle + 1;
		int i2 = f * gVertsPerTriangle + 2;

		for (int i = 0; i < gPosDim; ++i)
		{
			new_pos_data[i0 * gPosDim + i] = static_cast<float>(v0.mPosition[i]);
			new_pos_data[i1 * gPosDim + i] = static_cast<float>(v1.mPosition[i]);
			new_pos_data[i2 * gPosDim + i] = static_cast<float>(v2.mPosition[i]);
		}
		
		for (int i = 0; i < gNormDim; ++i)
		{
			new_norm_data[i0 * gNormDim + i] = static_cast<float>(normal[i]);
			new_norm_data[i1 * gNormDim + i] = static_cast<float>(normal[i]);
			new_norm_data[i2 * gNormDim + i] = static_cast<float>(normal[i]);
		}

		for (int i = 0; i < gCoordDim; ++i)
		{
			new_coord_data[i0 * gCoordDim + i] = v0.mCoord[i];
			new_coord_data[i1 * gCoordDim + i] = v1.mCoord[i];
			new_coord_data[i2 * gCoordDim + i] = v2.mCoord[i];
		}

		new_idx_data[f * gVertsPerTriangle] = i0;
		new_idx_data[f * gVertsPerTriangle + 1] = i1;
		new_idx_data[f * gVertsPerTriangle + 2] = i2;
	}

	out_mesh = std::unique_ptr<cDrawMesh>(new cDrawMesh());
	BuildDrawMesh(new_pos_data.data(), static_cast<int>(new_pos_data.size()), 
				new_norm_data.data(), static_cast<int>(new_norm_data.size()), 
				new_coord_data.data(), static_cast<int>(new_coord_data.size()), 
				new_idx_data.data(), static_cast<int>(new_idx_data.size()),
				out_mesh.get());
}

void cMeshUtil::RayTest(const tVector& start, const tVector& end, const cDrawMesh& mesh, std::vector<tRayTestResult>& out_result)
{
	out_result.clear();
	int num_faces = mesh.GetNumFaces();
	for (int f = 0; f < num_faces; ++f)
	{
		tFace face = GetFace(f, mesh);
		cMeshUtil::tVertex v0 = cMeshUtil::GetVertex(face[0], mesh);
		cMeshUtil::tVertex v1 = cMeshUtil::GetVertex(face[1], mesh);
		cMeshUtil::tVertex v2 = cMeshUtil::GetVertex(face[2], mesh);

		tVector hit_pos;
		bool hit = RayIntersectTriangle(start, end, v0.mPosition, v1.mPosition, v2.mPosition, hit_pos);
		if (hit)
		{
			tRayTestResult curr_result;
			curr_result.mFace = f;
			curr_result.mDist = (hit_pos - start).norm();
			curr_result.mIntersect = hit_pos;

			out_result.push_back(curr_result);
		}
	}
}


void cMeshUtil::BuildPointMesh(std::unique_ptr<cDrawMesh>& out_mesh)
{
	const int num_verts = 1;
	const int pos_len = num_verts * gPosDim;
	const int norm_len = num_verts * gNormDim;
	const int coord_len = num_verts * gCoordDim;
	const int idx_len = num_verts;

	const float vert_data[pos_len] =
	{
		0, 0, 0
	};

	const float norm_data[norm_len] =
	{
		0, 0, 1
	};

	const float coord_data[coord_len] = {
		0, 0
	};

	int idx_data[idx_len];
	for (int i = 0; i < idx_len; ++i)
	{
		idx_data[i] = i;
	}

	out_mesh = std::unique_ptr<cDrawMesh>(new cDrawMesh());
	BuildDrawMesh(vert_data, pos_len, norm_data, norm_len, coord_data, coord_len, idx_data, idx_len, out_mesh.get());
}

void cMeshUtil::BuildLineMesh(std::unique_ptr<cDrawMesh>& out_mesh)
{
	const int num_verts = 2;
	const int pos_len = num_verts * gPosDim;
	const int norm_len = num_verts * gNormDim;
	const int coord_len = num_verts * gCoordDim;
	const int idx_len = num_verts;

	const float vert_data[pos_len] =
	{
		0, 0, 0,
		1, 0, 0
	};

	const float norm_data[norm_len] =
	{
		0, 0, 1,
		0, 0, 1
	};

	const float coord_data[coord_len] = {
		0, 0,
		1, 0
	};

	int idx_data[idx_len];
	for (int i = 0; i < idx_len; ++i)
	{
		idx_data[i] = i;
	}

	out_mesh = std::unique_ptr<cDrawMesh>(new cDrawMesh());
	BuildDrawMesh(vert_data, pos_len, norm_data, norm_len, coord_data, coord_len, idx_data, idx_len, out_mesh.get());
}

void cMeshUtil::BuildQuadMesh(std::unique_ptr<cDrawMesh>& out_mesh)
{
	const int num_verts = 4;
	const int pos_len = num_verts  * gPosDim;
	const int norm_len = num_verts * gNormDim;
	const int coord_len = num_verts * gCoordDim;
	const int idx_len = num_verts;

	const float vert_data[pos_len] =
	{
		0, 0, 0,
		1, 0, 0,
		1, 1, 0,
		0, 1, 0
	};

	const float norm_data[norm_len] =
	{ 
		0, 0, 1,
		0, 0, 1,
		0, 0, 1,
		0, 0, 1
	};

	const float coord_data[coord_len] = { 
		0, 0,
		1, 0,
		1, 1,
		0, 1
	};

	int idx_data[idx_len];
	for (int i = 0; i < idx_len; ++i)
	{
		idx_data[i] = i;
	}

	out_mesh = std::unique_ptr<cDrawMesh>(new cDrawMesh());
	BuildDrawMesh(vert_data, pos_len, norm_data, norm_len, coord_data, coord_len, idx_data, idx_len, out_mesh.get());
}

void cMeshUtil::BuildBoxSolidMesh(std::unique_ptr<cDrawMesh>& out_mesh)
{
	const int num_faces = 6;
	const int vert_size = num_faces * 6 * gPosDim;
	const int norm_size = num_faces * 6 * gNormDim;
	const int coord_size = num_faces * 6 * gCoordDim;
	const int idx_size = num_faces * 6;

	const float vert_data[vert_size] = {
		0.5, 0.5, -0.5, // top
		-0.5, 0.5, -0.5,
		-0.5, 0.5, 0.5,
		-0.5, 0.5, 0.5,
		0.5, 0.5, 0.5,
		0.5, 0.5, -0.5,

		0.5, -0.5, 0.5, // bottom
		-0.5, -0.5, 0.5,
		-0.5, -0.5, -0.5,
		-0.5, -0.5, -0.5,
		0.5, -0.5, -0.5,
		0.5, -0.5, 0.5,

		0.5, -0.5, 0.5, // front
		0.5, -0.5, -0.5,
		0.5, 0.5, -0.5,
		0.5, 0.5, -0.5,
		0.5, 0.5, 0.5,
		0.5, -0.5, 0.5,

		-0.5, -0.5, -0.5, // back
		-0.5, -0.5, 0.5,
		-0.5, 0.5, 0.5,
		-0.5, 0.5, 0.5,
		-0.5, 0.5, -0.5,
		-0.5, -0.5, -0.5,

		-0.5, -0.5, -0.5, // left
		-0.5, 0.5, -0.5,
		0.5, 0.5, -0.5,
		0.5, 0.5, -0.5,
		0.5, -0.5, -0.5,
		-0.5, -0.5, -0.5,

		0.5, -0.5, 0.5, // right
		0.5, 0.5, 0.5,
		-0.5, 0.5, 0.5,
		-0.5, 0.5, 0.5,
		-0.5, -0.5, 0.5,
		0.5, -0.5, 0.5
	};

	const float norm_data[vert_size] = {
		0, 1, 0, // top
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,

		0, -1, 0, // bottom
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,

		1, 0, 0, // front
		1, 0, 0,
		1, 0, 0,
		1, 0, 0,
		1, 0, 0,
		1, 0, 0,

		-1, 0, 0, // back
		-1, 0, 0,
		-1, 0, 0,
		-1, 0, 0,
		-1, 0, 0,
		-1, 0, 0,

		0, 0, -1, // left
		0, 0, -1,
		0, 0, -1,
		0, 0, -1,
		0, 0, -1,
		0, 0, -1,

		0, 0, 1, // right
		0, 0, 1,
		0, 0, 1,
		0, 0, 1,
		0, 0, 1,
		0, 0, 1,
	};


	const float coord_data[coord_size] = {
		0, 0, // top
		1, 0,
		1, 1,
		1, 1,
		0, 1,
		0, 0,

		1, 0, // bottom
		1, 1,
		0, 1,
		0, 1,
		0, 0,
		1, 0,

		0, 0, // front
		1, 0,
		1, 1,
		1, 1,
		0, 1,
		0, 0,

		0, 0, // back
		1, 0,
		1, 1,
		1, 1,
		0, 1,
		0, 0,

		1, 0, // left
		1, 1,
		0, 1,
		0, 1,
		0, 0,
		1, 0,

		1, 0, // right
		1, 1,
		0, 1,
		0, 1,
		0, 0,
		1, 0
	};

	int idx_data[idx_size];
	for (int i = 0; i < idx_size; ++i)
	{
		idx_data[i] = i;
	}

	out_mesh = std::unique_ptr<cDrawMesh>(new cDrawMesh());
	BuildDrawMesh(vert_data, vert_size, norm_data, norm_size, coord_data, coord_size, idx_data, idx_size, out_mesh.get());
}

void cMeshUtil::BuildBoxWireMesh(std::unique_ptr<cDrawMesh>& out_mesh)
{
	const int num_edges = 12;
	const int vert_size = num_edges * 2 * gPosDim;
	const int norm_size = num_edges * 2 * gNormDim;
	const int coord_size = num_edges * 2 * gCoordDim;
	const int idx_size = num_edges * 2;

	const float vert_data[vert_size] = {
		0.5, 0.5, -0.5, // top
		-0.5, 0.5, -0.5,
		-0.5, 0.5, -0.5,
		-0.5, 0.5, 0.5,
		-0.5, 0.5, 0.5,
		0.5, 0.5, 0.5,
		0.5, 0.5, 0.5,
		0.5, 0.5, -0.5,

		0.5, -0.5, 0.5, // bottom
		-0.5, -0.5, 0.5,
		-0.5, -0.5, 0.5,
		-0.5, -0.5, -0.5,
		-0.5, -0.5, -0.5,
		0.5, -0.5, -0.5,
		0.5, -0.5, -0.5,
		0.5, -0.5, 0.5,

		0.5, -0.5, 0.5, // front
		0.5, 0.5, 0.5,

		-0.5, -0.5, -0.5, // back
		-0.5, 0.5, -0.5,
		
		0.5, 0.5, -0.5, // left
		0.5, -0.5, -0.5,
		
		-0.5, 0.5, 0.5, // right
		-0.5, -0.5, 0.5
	};

	const float norm_data[vert_size] = {
		0, 1, 0, // top
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,

		0, -1, 0, // bottom
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,

		1, 0, 0, // front
		1, 0, 0,

		-1, 0, 0, // back
		-1, 0, 0,

		0, 0, -1, // left
		0, 0, -1,

		0, 0, 1, // right
		0, 0, 1
	};


	const float coord_data[vert_size] = {
		0, 0, // top
		1, 0,
		1, 0,
		1, 1,
		1, 1,
		0, 1,
		0, 1,
		0, 0,

		1, 0, // bottom
		1, 1,
		1, 1,
		0, 1,
		0, 1,
		0, 0,
		0, 0,
		1, 0,

		0, 0, // front
		0, 1,
		
		0, 0, // back
		0, 1,

		0, 1, // left
		0, 0,

		0, 1, // right
		0, 0
	};

	int idx_data[idx_size];
	for (int i = 0; i < idx_size; ++i)
	{
		idx_data[i] = i;
	}

	out_mesh = std::unique_ptr<cDrawMesh>(new cDrawMesh());
	BuildDrawMesh(vert_data, vert_size, norm_data, norm_size, coord_data, coord_size, idx_data, idx_size, out_mesh.get());
}

void cMeshUtil::BuildSphereMesh(int stacks, int slices, std::unique_ptr<cDrawMesh>& out_mesh)
{
	const float r = 1;
	int num_triangles = 2 * slices * (stacks - 1);

	std::vector<float> pos_data;
	std::vector<float> norm_data;
	std::vector<float> coord_data;
	std::vector<int> idx_data;
	pos_data.reserve(num_triangles * gVertsPerTriangle * gPosDim);
	norm_data.reserve(num_triangles * gVertsPerTriangle * gNormDim);
	coord_data.reserve(num_triangles * gVertsPerTriangle * gCoordDim);
	idx_data.resize(num_triangles * gVertsPerTriangle);

	for (int i = 0; i < stacks; ++i)
	{
		float rho0 = 0.5 * M_PI - (i * M_PI) / stacks;
		float rho1 = 0.5 * M_PI - ((i + 1) * M_PI) / stacks;
		float coord_v0 = 1 - static_cast<float>(i) / stacks;
		float coord_v1 = 1 - static_cast<float>(i + 1) / stacks;

		float y0 = r * std::sin(rho0);
		float r0 = r * std::cos(rho0);
		float y1 = r * std::sin(rho1);
		float r1 = r * std::cos(rho1);
		float y2 = y1;
		float y3 = y0;

		for (int j = 0; j < slices; ++j)
		{
			float theta0 = (j * 2 * M_PI) / slices;
			float theta1 = ((j + 1) * 2 * M_PI) / slices;
			float coord_u0 = static_cast<float>(j) / slices;
			float coord_u1 = static_cast<float>(j + 1) / slices;

			float x0 = r0 * std::cos(theta0);
			float z0 = r0 * std::sin(-theta0);
			float u0 = coord_u0;
			float v0 = coord_v0;
			float x1 = r1 * std::cos(theta0);
			float z1 = r1 * std::sin(-theta0);
			float u1 = coord_u0;
			float v1 = coord_v1;
			float x2 = r1 * std::cos(theta1);
			float z2 = r1 * std::sin(-theta1);
			float u2 = coord_u1;
			float v2 = coord_v1;
			float x3 = r0 * std::cos(theta1);
			float z3 = r0 * std::sin(-theta1);
			float u3 = coord_u1;
			float v3 = coord_v0;

			if (i != stacks - 1)
			{
				pos_data.push_back(x0);
				pos_data.push_back(y0);
				pos_data.push_back(z0);
				norm_data.push_back(x0);
				norm_data.push_back(y0);
				norm_data.push_back(z0);
				coord_data.push_back(u0);
				coord_data.push_back(v0);

				pos_data.push_back(x1);
				pos_data.push_back(y1);
				pos_data.push_back(z1);
				norm_data.push_back(x1);
				norm_data.push_back(y1);
				norm_data.push_back(z1);
				coord_data.push_back(u1);
				coord_data.push_back(v1);

				pos_data.push_back(x2);
				pos_data.push_back(y2);
				pos_data.push_back(z2);
				norm_data.push_back(x2);
				norm_data.push_back(y2);
				norm_data.push_back(z2);
				coord_data.push_back(u2);
				coord_data.push_back(v2);
			}
			
			if (i != 0)
			{
				pos_data.push_back(x2);
				pos_data.push_back(y2);
				pos_data.push_back(z2);
				norm_data.push_back(x2);
				norm_data.push_back(y2);
				norm_data.push_back(z2);
				coord_data.push_back(u2);
				coord_data.push_back(v2);

				pos_data.push_back(x3);
				pos_data.push_back(y3);
				pos_data.push_back(z3);
				norm_data.push_back(x3);
				norm_data.push_back(y3);
				norm_data.push_back(z3);
				coord_data.push_back(u3);
				coord_data.push_back(v3);

				pos_data.push_back(x0);
				pos_data.push_back(y0);
				pos_data.push_back(z0);
				norm_data.push_back(x0);
				norm_data.push_back(y0);
				norm_data.push_back(z0);
				coord_data.push_back(u0);
				coord_data.push_back(v0);
			}
		}
	}

	for (int i = 0; i < static_cast<int>(idx_data.size()); ++i)
	{
		idx_data[i] = i;
	}

	out_mesh = std::unique_ptr<cDrawMesh>(new cDrawMesh());
	BuildDrawMesh(pos_data.data(), static_cast<int>(pos_data.size()), 
					norm_data.data(), static_cast<int>(norm_data.size()),
					coord_data.data(), static_cast<int>(coord_data.size()),
					idx_data.data(), static_cast<int>(idx_data.size()), out_mesh.get());
}

void cMeshUtil::BuildSphereWireSimpleMesh(int stacks, int slices, std::unique_ptr<cDrawMesh>& out_mesh)
{
	const float r = 1;
	int slices0 = 4;
	int num_lines = slices0 * stacks + slices;

	std::vector<float> pos_data;
	std::vector<float> norm_data;
	std::vector<float> coord_data;
	std::vector<int> idx_data;
	pos_data.resize(num_lines * gVertsPerLine * gPosDim);
	norm_data.resize(num_lines * gVertsPerLine * gNormDim);
	coord_data.resize(num_lines * gVertsPerLine * gCoordDim);
	idx_data.resize(num_lines * gVertsPerLine);

	for (int i = 0; i < stacks; ++i)
	{
		float rho0 = 0.5 * M_PI - (i * M_PI) / stacks;
		float rho1 = 0.5 * M_PI - ((i + 1) * M_PI) / stacks;
		float coord_v0 = 1 - static_cast<float>(i) / stacks;
		float coord_v1 = 1 - static_cast<float>(i + 1) / stacks;

		float y0 = r * std::sin(rho0);
		float r0 = r * std::cos(rho0);
		float y1 = r * std::sin(rho1);
		float r1 = r * std::cos(rho1);

		for (int j = 0; j < slices0; ++j)
		{
			float theta0 = (j * 2 * M_PI) / slices0;
			float coord_u0 = static_cast<float>(j) / slices0;

			float x0 = r0 * std::cos(theta0);
			float z0 = r0 * std::sin(-theta0);
			float u0 = coord_u0;
			float v0 = coord_v0;
			float x1 = r1 * std::cos(theta0);
			float z1 = r1 * std::sin(-theta0);
			float u1 = coord_u0;
			float v1 = coord_v1;

			int pos_offset = (i * slices0 + j) * gVertsPerLine * gPosDim;
			int norm_offset = (i * slices0 + j) * gVertsPerLine * gNormDim;
			int coord_offset = (i * slices0 + j) * gVertsPerLine * gCoordDim;

			if (i >= stacks / 2)
			{
				pos_offset += slices * gVertsPerLine * gPosDim;
				norm_offset += slices * gVertsPerLine * gNormDim;
				coord_offset += slices * gVertsPerLine * gCoordDim;
			}

			pos_data[pos_offset + 0] = x0;
			pos_data[pos_offset + 1] = y0;
			pos_data[pos_offset + 2] = z0;
			norm_data[norm_offset + 0] = x0;
			norm_data[norm_offset + 1] = y0;
			norm_data[norm_offset + 2] = z0;
			coord_data[coord_offset + 0] = u0;
			coord_data[coord_offset + 1] = v0;

			pos_data[pos_offset + 3] = x1;
			pos_data[pos_offset + 4] = y1;
			pos_data[pos_offset + 5] = z1;
			norm_data[norm_offset + 3] = x1;
			norm_data[norm_offset + 4] = y1;
			norm_data[norm_offset + 5] = z1;
			coord_data[coord_offset + 2] = u1;
			coord_data[coord_offset + 3] = v1;
		}
	}

	for (int j = 0; j < slices; ++j)
	{
		float theta0 = (j * 2 * M_PI) / slices;
		float theta1 = ((j + 1) * 2 * M_PI) / slices;
		float coord_u0 = static_cast<float>(j) / slices;
		float coord_v0 = 0.5;
		float coord_v1 = 0.5;

		float x0 = r * std::cos(theta0);
		float y0 = 0;
		float z0 = r * std::sin(-theta0);
		float u0 = coord_u0;
		float v0 = coord_v0;
		float x1 = r * std::cos(theta1);
		float y1 = 0;
		float z1 = r * std::sin(-theta1);
		float u1 = coord_u0;
		float v1 = coord_v1;

		int pos_offset = (stacks / 2 * slices0 + j) * gVertsPerLine * gPosDim;
		int norm_offset = (stacks / 2 * slices0 + j) * gVertsPerLine * gNormDim;
		int coord_offset = (stacks / 2 * slices0 + j) * gVertsPerLine * gCoordDim;

		pos_data[pos_offset + 0] = x0;
		pos_data[pos_offset + 1] = y0;
		pos_data[pos_offset + 2] = z0;
		norm_data[norm_offset + 0] = x0;
		norm_data[norm_offset + 1] = y0;
		norm_data[norm_offset + 2] = z0;
		coord_data[coord_offset + 0] = u0;
		coord_data[coord_offset + 1] = v0;

		pos_data[pos_offset + 3] = x1;
		pos_data[pos_offset + 4] = y1;
		pos_data[pos_offset + 5] = z1;
		norm_data[norm_offset + 3] = x1;
		norm_data[norm_offset + 4] = y1;
		norm_data[norm_offset + 5] = z1;
		coord_data[coord_offset + 2] = u1;
		coord_data[coord_offset + 3] = v1;
	}

	for (int i = 0; i < static_cast<int>(idx_data.size()); ++i)
	{
		idx_data[i] = i;
	}

	out_mesh = std::unique_ptr<cDrawMesh>(new cDrawMesh());
	BuildDrawMesh(pos_data.data(), static_cast<int>(pos_data.size()),
		norm_data.data(), static_cast<int>(norm_data.size()),
		coord_data.data(), static_cast<int>(coord_data.size()),
		idx_data.data(), static_cast<int>(idx_data.size()), out_mesh.get());
}

void cMeshUtil::BuildDiskMesh(int slices, std::unique_ptr<cDrawMesh>& out_mesh)
{
	const int num_verts = 2 + slices;

	std::vector<float> pos_data(num_verts * gPosDim);
	std::vector<float> norm_data(num_verts * gNormDim);
	std::vector<float> coord_data(num_verts * gCoordDim);
	std::vector<int> idx_data(num_verts);

	pos_data[0] = 0;
	pos_data[1] = 0;
	pos_data[2] = 0;
	norm_data[0] = 0;
	norm_data[1] = 0;
	norm_data[2] = 1;
	coord_data[0] = 0;
	coord_data[1] = 0;

	for (int i = 0; i <= slices; ++i)
	{
		float theta = (i * 2 * M_PI) / slices;
		int pos_offset = (i + 1) * gPosDim;
		int norm_offset = (i + 1) * gNormDim;
		int coord_offset = (i + 1) * gCoordDim;

		pos_data[pos_offset + 0] = std::cos(theta);
		pos_data[pos_offset + 1] = std::sin(theta);
		pos_data[pos_offset + 2] = 0;
		norm_data[norm_offset + 0] = 0;
		norm_data[norm_offset + 1] = 0;
		norm_data[norm_offset + 2] = 1;
		coord_data[coord_offset + 0] = 0;
		coord_data[coord_offset + 1] = 0;
	}

	for (int i = 0; i < static_cast<int>(idx_data.size()); ++i)
	{
		idx_data[i] = i;
	}

	out_mesh = std::unique_ptr<cDrawMesh>(new cDrawMesh());
	BuildDrawMesh(pos_data.data(), static_cast<int>(pos_data.size()),
					norm_data.data(), static_cast<int>(norm_data.size()),
					coord_data.data(), static_cast<int>(coord_data.size()),
					idx_data.data(), static_cast<int>(idx_data.size()), out_mesh.get());
}

void cMeshUtil::BuildTriangleMesh(std::unique_ptr<cDrawMesh>& out_mesh)
{
	const float side_len = 1;
	const float h = std::sqrt(0.75 * side_len * side_len);
	const int num_verts = 3;
	const int vert_len = num_verts * gPosDim;
	const int norm_len = num_verts * gNormDim;
	const int coord_len = num_verts * gCoordDim;
	const int idx_len = num_verts;

	const float vert_data[vert_len] =
	{
		-0.5f * side_len, -0.5f * h, 0.f,
		0.5f * side_len, -0.5f * h, 0.f,
		0.f, 0.5f * h, 0.f
	};

	const float norm_data[norm_len] =
	{
		0, 0, 1,
		0, 0, 1,
		0, 0, 1
	};


	const float coord_data[coord_len] = {
		0, 0,
		1, 0,
		0.5, 1
	};

	int idx_data[idx_len] = { 0, 1, 2 };

	out_mesh = std::unique_ptr<cDrawMesh>(new cDrawMesh());
	BuildDrawMesh(vert_data, vert_len, norm_data, norm_len, coord_data, coord_len, idx_data, idx_len, out_mesh.get());
}

void cMeshUtil::BuildCylinder(int slices, std::unique_ptr<cDrawMesh>& out_mesh)
{
	const float r = 1.f;
	const float h = 1.f;
	const int num_verts = 12 * slices;

	std::vector<float> pos_data(num_verts * gPosDim);
	std::vector<float> norm_data(num_verts * gNormDim);
	std::vector<float> coord_data(num_verts * gCoordDim);
	std::vector<int> idx_data(num_verts);
	
	for (int i = 0; i < slices; ++i)
	{
		double theta0 = (i * 2 * M_PI) / slices;
		double theta1 = ((i + 1) * 2 * M_PI) / slices;

		double x0 = r * std::cos(theta0);
		double z0 = r * std::sin(-theta0);
		double u0 = static_cast<float>(i) / slices;

		double x1 = r * std::cos(theta1);
		double z1 = r * std::sin(-theta1);
		double u1 = static_cast<float>(i + 1) / slices;

		tVector n0 = tVector(x0, 0, z0, 0).normalized();
		tVector n1 = tVector(x1, 0, z1, 0).normalized();

		int pos_offset = i * 6 * gPosDim;
		int norm_offset = i * 6 * gNormDim;
		int coord_offset = i * 6 * gCoordDim;

		pos_data[pos_offset] = x0;
		pos_data[pos_offset + 1] = -0.5 * h;
		pos_data[pos_offset + 2] = z0;
		pos_data[pos_offset + 3] = x1;
		pos_data[pos_offset + 4] = -0.5 * h;
		pos_data[pos_offset + 5] = z1;
		pos_data[pos_offset + 6] = x1;
		pos_data[pos_offset + 7] = 0.5 * h;
		pos_data[pos_offset + 8] = z1;
		pos_data[pos_offset + 9] = x1;
		pos_data[pos_offset + 10] = 0.5 * h;
		pos_data[pos_offset + 11] = z1;
		pos_data[pos_offset + 12] = x0;
		pos_data[pos_offset + 13] = 0.5 * h;
		pos_data[pos_offset + 14] = z0;
		pos_data[pos_offset + 15] = x0;
		pos_data[pos_offset + 16] = -0.5 * h;
		pos_data[pos_offset + 17] = z0;

		norm_data[norm_offset] = n0[0];
		norm_data[norm_offset + 1] = n0[1];
		norm_data[norm_offset + 2] = n0[2];
		norm_data[norm_offset + 3] = n1[0];
		norm_data[norm_offset + 4] = n1[1];
		norm_data[norm_offset + 5] = n1[2];
		norm_data[norm_offset + 6] = n1[0];
		norm_data[norm_offset + 7] = n1[1];
		norm_data[norm_offset + 8] = n1[2];
		norm_data[norm_offset + 9] = n1[0];
		norm_data[norm_offset + 10] = n1[1];
		norm_data[norm_offset + 11] = n1[2];
		norm_data[norm_offset + 12] = n0[0];
		norm_data[norm_offset + 13] = n0[1];
		norm_data[norm_offset + 14] = n0[2];
		norm_data[norm_offset + 15] = n0[0];
		norm_data[norm_offset + 16] = n0[1];
		norm_data[norm_offset + 17] = n0[2];

		coord_data[coord_offset] = u0;
		coord_data[coord_offset + 1] = 0.f;
		coord_data[coord_offset + 2] = u1;
		coord_data[coord_offset + 3] = 0.f;
		coord_data[coord_offset + 4] = u1;
		coord_data[coord_offset + 5] = 1;
		coord_data[coord_offset + 6] = u1;
		coord_data[coord_offset + 7] = 1.f;
		coord_data[coord_offset + 8] = u0;
		coord_data[coord_offset + 9] = 1.f;
		coord_data[coord_offset + 10] = u0;
		coord_data[coord_offset + 11] = 0.f;

		pos_offset = (slices * 6 + i * 6) * gPosDim;
		norm_offset = (slices * 6 + i * 6) * gNormDim;
		coord_offset = (slices * 6 + i * 6) * gCoordDim;

		pos_data[pos_offset] = x0;
		pos_data[pos_offset + 1] = 0.5 * h;
		pos_data[pos_offset + 2] = z0;
		pos_data[pos_offset + 3] = x1;
		pos_data[pos_offset + 4] = 0.5 * h;
		pos_data[pos_offset + 5] = z1;
		pos_data[pos_offset + 6] = 0.f;
		pos_data[pos_offset + 7] = 0.5 * h;
		pos_data[pos_offset + 8] = 0.f;
		pos_data[pos_offset + 9] = 0.f;
		pos_data[pos_offset + 10] = -0.5 * h;
		pos_data[pos_offset + 11] = 0.f;
		pos_data[pos_offset + 12] = x1;
		pos_data[pos_offset + 13] = -0.5 * h;
		pos_data[pos_offset + 14] = z1;
		pos_data[pos_offset + 15] = x0;
		pos_data[pos_offset + 16] = -0.5 * h;
		pos_data[pos_offset + 17] = z0;

		norm_data[norm_offset] = 0.f;
		norm_data[norm_offset + 1] = 1.f;
		norm_data[norm_offset + 2] = 0.f;
		norm_data[norm_offset + 3] = 0.f;
		norm_data[norm_offset + 4] = 1.f;
		norm_data[norm_offset + 5] = 0.f;
		norm_data[norm_offset + 6] = 0.f;
		norm_data[norm_offset + 7] = 1.f;
		norm_data[norm_offset + 8] = 0.f;
		norm_data[norm_offset + 9] = 0.f;
		norm_data[norm_offset + 10] = -1.f;
		norm_data[norm_offset + 11] = 0.f;
		norm_data[norm_offset + 12] = 0.f;
		norm_data[norm_offset + 13] = -1.f;
		norm_data[norm_offset + 14] = 0.f;
		norm_data[norm_offset + 15] = 0.f;
		norm_data[norm_offset + 16] = -1.f;
		norm_data[norm_offset + 17] = 0.f;

		coord_data[coord_offset] = u0;
		coord_data[coord_offset + 1] = 1.f;
		coord_data[coord_offset + 2] = u1;
		coord_data[coord_offset + 3] = 1.f;
		coord_data[coord_offset + 4] = u1;
		coord_data[coord_offset + 5] = 1.f;
		coord_data[coord_offset + 6] = u0;
		coord_data[coord_offset + 7] = 0.f;
		coord_data[coord_offset + 8] = u1;
		coord_data[coord_offset + 9] = 0.f;
		coord_data[coord_offset + 10] = u0;
		coord_data[coord_offset + 11] = 0.f;
	}

	for (int i = 0; i < static_cast<int>(idx_data.size()); ++i)
	{
		idx_data[i] = i;
	}

	out_mesh = std::unique_ptr<cDrawMesh>(new cDrawMesh());
	BuildDrawMesh(pos_data.data(), static_cast<int>(pos_data.size()),
				norm_data.data(), static_cast<int>(norm_data.size()),
				coord_data.data(), static_cast<int>(coord_data.size()),
				idx_data.data(), static_cast<int>(idx_data.size()), out_mesh.get());
}

void cMeshUtil::BuildCylinderWireSimple(int slices, std::unique_ptr<cDrawMesh>& out_mesh)
{
	const float r = 1.f;
	const float h = 1.f;
	const int slices0 = 4;
	const int num_verts = 4 * slices + 2 * slices0;

	std::vector<float> pos_data(num_verts * gPosDim);
	std::vector<float> norm_data(num_verts * gNormDim);
	std::vector<float> coord_data(num_verts * gCoordDim);
	std::vector<int> idx_data(num_verts);
	
	for (int i = 0; i < slices; ++i)
	{
		double theta0 = (i * 2 * M_PI) / slices;
		double theta1 = ((i + 1) * 2 * M_PI) / slices;

		double x0 = r * std::cos(theta0);
		double z0 = r * std::sin(-theta0);
		double u0 = static_cast<float>(i) / slices;

		double x1 = r * std::cos(theta1);
		double z1 = r * std::sin(-theta1);
		double u1 = static_cast<float>(i + 1) / slices;

		tVector n0 = tVector(0, -1, 0, 0).normalized();
		tVector n1 = tVector(0, 1, 0, 0).normalized();

		int pos_offset = i * 4 * gPosDim;
		int norm_offset = i * 4 * gNormDim;
		int coord_offset = i * 4 * gCoordDim;

		pos_data[pos_offset] = x0;
		pos_data[pos_offset + 1] = -0.5 * h;
		pos_data[pos_offset + 2] = z0;
		pos_data[pos_offset + 3] = x1;
		pos_data[pos_offset + 4] = -0.5 * h;
		pos_data[pos_offset + 5] = z1;

		norm_data[norm_offset] = n0[0];
		norm_data[norm_offset + 1] = n0[1];
		norm_data[norm_offset + 2] = n0[2];
		norm_data[norm_offset + 3] = n0[0];
		norm_data[norm_offset + 4] = n0[1];
		norm_data[norm_offset + 5] = n0[2];

		coord_data[coord_offset] = u0;
		coord_data[coord_offset + 1] = 0.f;
		coord_data[coord_offset + 2] = u1;
		coord_data[coord_offset + 3] = 0.f;

		pos_data[pos_offset + 6] = x0;
		pos_data[pos_offset + 7] = 0.5 * h;
		pos_data[pos_offset + 8] = z0;
		pos_data[pos_offset + 9] = x1;
		pos_data[pos_offset + 10] = 0.5 * h;
		pos_data[pos_offset + 11] = z1;

		norm_data[norm_offset + 6] = n1[0];
		norm_data[norm_offset + 7] = n1[1];
		norm_data[norm_offset + 8] = n1[2];
		norm_data[norm_offset + 9] = n1[0];
		norm_data[norm_offset + 10] = n1[1];
		norm_data[norm_offset + 11] = n1[2];

		coord_data[coord_offset + 4] = u0;
		coord_data[coord_offset + 5] = 1.f;
		coord_data[coord_offset + 6] = u1;
		coord_data[coord_offset + 7] = 1.f;
	}

	for (int i = 0; i < slices0; ++i)
	{
		double theta0 = (i * 2 * M_PI) / slices0;

		double x0 = r * std::cos(theta0);
		double z0 = r * std::sin(-theta0);
		double u0 = static_cast<float>(i) / slices0;

		tVector n0 = tVector(x0, 0, z0, 0).normalized();

		int pos_offset = (slices * 4 + i * 2) * gPosDim;
		int norm_offset = (slices * 4 + i * 2) * gNormDim;
		int coord_offset = (slices * 4 + i * 2) * gCoordDim;

		pos_data[pos_offset] = x0;
		pos_data[pos_offset + 1] = -0.5 * h;
		pos_data[pos_offset + 2] = z0;
		pos_data[pos_offset + 3] = x0;
		pos_data[pos_offset + 4] = 0.5 * h;
		pos_data[pos_offset + 5] = z0;

		norm_data[norm_offset] = n0[0];
		norm_data[norm_offset + 1] = n0[1];
		norm_data[norm_offset + 2] = n0[2];
		norm_data[norm_offset + 3] = n0[0];
		norm_data[norm_offset + 4] = n0[1];
		norm_data[norm_offset + 5] = n0[2];

		coord_data[coord_offset] = u0;
		coord_data[coord_offset + 1] = 0.f;
		coord_data[coord_offset + 2] = u0;
		coord_data[coord_offset + 3] = 1.f;
	}

	for (int i = 0; i < static_cast<int>(idx_data.size()); ++i)
	{
		idx_data[i] = i;
	}

	out_mesh = std::unique_ptr<cDrawMesh>(new cDrawMesh());
	BuildDrawMesh(pos_data.data(), static_cast<int>(pos_data.size()),
				norm_data.data(), static_cast<int>(norm_data.size()),
				coord_data.data(), static_cast<int>(coord_data.size()),
				idx_data.data(), static_cast<int>(idx_data.size()), out_mesh.get());
}

void cMeshUtil::BuildConeWireSimple(int slices, std::unique_ptr<cDrawMesh>& out_mesh)
{
	const float r = 1.f;
	const float h = 1.f;
	const int slices0 = 4;
	const int num_verts = 2 * slices + 2 * slices0;

	std::vector<float> pos_data(num_verts * gPosDim);
	std::vector<float> norm_data(num_verts * gNormDim);
	std::vector<float> coord_data(num_verts * gCoordDim);
	std::vector<int> idx_data(num_verts);

	for (int i = 0; i < slices; ++i)
	{
		double theta0 = (i * 2 * M_PI) / slices;
		double theta1 = ((i + 1) * 2 * M_PI) / slices;

		double x0 = r * std::cos(theta0);
		double z0 = r * std::sin(-theta0);
		double u0 = static_cast<float>(i) / slices;

		double x1 = r * std::cos(theta1);
		double z1 = r * std::sin(-theta1);
		double u1 = static_cast<float>(i + 1) / slices;

		tVector n0 = tVector(0, -1, 0, 0).normalized();

		int pos_offset = i * 2 * gPosDim;
		int norm_offset = i * 2 * gNormDim;
		int coord_offset = i * 2 * gCoordDim;

		pos_data[pos_offset] = x0;
		pos_data[pos_offset + 1] = 0;
		pos_data[pos_offset + 2] = z0;
		pos_data[pos_offset + 3] = x1;
		pos_data[pos_offset + 4] = 0;
		pos_data[pos_offset + 5] = z1;

		norm_data[norm_offset] = n0[0];
		norm_data[norm_offset + 1] = n0[1];
		norm_data[norm_offset + 2] = n0[2];
		norm_data[norm_offset + 3] = n0[0];
		norm_data[norm_offset + 4] = n0[1];
		norm_data[norm_offset + 5] = n0[2];

		coord_data[coord_offset] = u0;
		coord_data[coord_offset + 1] = 0.f;
		coord_data[coord_offset + 2] = u1;
		coord_data[coord_offset + 3] = 0.f;
	}

	for (int i = 0; i < slices0; ++i)
	{
		double theta0 = (i * 2 * M_PI) / slices0;

		double x0 = r * std::cos(theta0);
		double z0 = r * std::sin(-theta0);
		double u0 = static_cast<float>(i) / slices0;

		tVector n0 = tVector(h * std::cos(theta0), r, -h * std::sin(theta0), 0).normalized();

		int pos_offset = (slices * 2 + i * 2) * gPosDim;
		int norm_offset = (slices * 2 + i * 2) * gNormDim;
		int coord_offset = (slices * 2 + i * 2) * gCoordDim;

		pos_data[pos_offset] = x0;
		pos_data[pos_offset + 1] = 0;
		pos_data[pos_offset + 2] = z0;
		pos_data[pos_offset + 3] = 0;
		pos_data[pos_offset + 4] = h;
		pos_data[pos_offset + 5] = 0;

		norm_data[norm_offset] = n0[0];
		norm_data[norm_offset + 1] = n0[1];
		norm_data[norm_offset + 2] = n0[2];
		norm_data[norm_offset + 3] = n0[0];
		norm_data[norm_offset + 4] = n0[1];
		norm_data[norm_offset + 5] = n0[2];

		coord_data[coord_offset] = u0;
		coord_data[coord_offset + 1] = 0.f;
		coord_data[coord_offset + 2] = u0;
		coord_data[coord_offset + 3] = 1.f;
	}

	for (int i = 0; i < static_cast<int>(idx_data.size()); ++i)
	{
		idx_data[i] = i;
	}

	out_mesh = std::unique_ptr<cDrawMesh>(new cDrawMesh());
	BuildDrawMesh(pos_data.data(), static_cast<int>(pos_data.size()),
		norm_data.data(), static_cast<int>(norm_data.size()),
		coord_data.data(), static_cast<int>(coord_data.size()),
		idx_data.data(), static_cast<int>(idx_data.size()), out_mesh.get());
}

void cMeshUtil::BuildCone(int slices, std::unique_ptr<cDrawMesh>& out_mesh)
{
	const float r = 1.f;
	const float h = 1.f;
	const int num_verts = 6 * slices;

	std::vector<float> pos_data(num_verts * gPosDim);
	std::vector<float> norm_data(num_verts * gNormDim);
	std::vector<float> coord_data(num_verts * gCoordDim);
	std::vector<int> idx_data(num_verts);

	for (int i = 0; i < slices; ++i)
	{
		double theta0 = (i * 2 * M_PI) / slices;
		double theta1 = ((i + 1) * 2 * M_PI) / slices;

		double x0 = r * std::cos(theta0);
		double z0 = r * std::sin(-theta0);
		double u0 = static_cast<double>(i) / slices;

		double x1 = r * std::cos(theta1);
		double z1 = r * std::sin(-theta1);
		double u1 = static_cast<double>(i + 1) / slices;

		double u2 = (i + 0.5) / slices;

		tVector n0 = tVector(h * std::cos(theta0), r, -h * std::sin(theta0), 0).normalized();
		tVector n1 = tVector(h * std::cos(theta1), r, -h * std::sin(theta1), 0).normalized();
		tVector n2 = tVector(h * std::cos(0.5 * (theta0 + theta1)), r, -h * std::sin(0.5 * (theta0 + theta1)), 0).normalized();

		int pos_offset = i * 3 * gPosDim;
		int norm_offset = i * 3 * gNormDim;
		int coord_offset = i * 3 * gCoordDim;

		pos_data[pos_offset] = 0;
		pos_data[pos_offset + 1] = h;
		pos_data[pos_offset + 2] = 0;
		pos_data[pos_offset + 3] = x0;
		pos_data[pos_offset + 4] = 0;
		pos_data[pos_offset + 5] = z0;
		pos_data[pos_offset + 6] = x1;
		pos_data[pos_offset + 7] = 0;
		pos_data[pos_offset + 8] = z1;
		
		norm_data[norm_offset] = n2[0];
		norm_data[norm_offset + 1] = n2[1];
		norm_data[norm_offset + 2] = n2[2];
		norm_data[norm_offset + 3] = n0[0];
		norm_data[norm_offset + 4] = n0[1];
		norm_data[norm_offset + 5] = n0[2];
		norm_data[norm_offset + 6] = n1[0];
		norm_data[norm_offset + 7] = n1[1];
		norm_data[norm_offset + 8] = n1[2];

		coord_data[coord_offset] = u2;
		coord_data[coord_offset + 1] = 1.f;
		coord_data[coord_offset + 2] = u0;
		coord_data[coord_offset + 3] = 0.f;
		coord_data[coord_offset + 4] = u1;
		coord_data[coord_offset + 5] = 0.f;

		pos_offset = (slices * 3 + i * 3) * gPosDim;
		norm_offset = (slices * 3 + i * 3) * gNormDim;
		coord_offset = (slices * 3 + i * 3) * gCoordDim;

		pos_data[pos_offset] = x1;
		pos_data[pos_offset + 1] = 0;
		pos_data[pos_offset + 2] = z1;
		pos_data[pos_offset + 3] = x0;
		pos_data[pos_offset + 4] = 0;
		pos_data[pos_offset + 5] = z0;
		pos_data[pos_offset + 6] = 0;
		pos_data[pos_offset + 7] = 0;
		pos_data[pos_offset + 8] = 0;

		norm_data[norm_offset] = 0.f;
		norm_data[norm_offset + 1] = -1.f;
		norm_data[norm_offset + 2] = 0.f;
		norm_data[norm_offset + 3] = 0.f;
		norm_data[norm_offset + 4] = -1.f;
		norm_data[norm_offset + 5] = 0.f;
		norm_data[norm_offset + 6] = 0.f;
		norm_data[norm_offset + 7] = -1.f;
		norm_data[norm_offset + 8] = 0.f;

		coord_data[coord_offset] = u1;
		coord_data[coord_offset + 1] = 0.f;
		coord_data[coord_offset + 2] = u0;
		coord_data[coord_offset + 3] = 0.f;
		coord_data[coord_offset + 4] = u2;
		coord_data[coord_offset + 5] = 0.f;
	}

	for (int i = 0; i < static_cast<int>(idx_data.size()); ++i)
	{
		idx_data[i] = i;
	}

	out_mesh = std::unique_ptr<cDrawMesh>(new cDrawMesh());
	BuildDrawMesh(pos_data.data(), static_cast<int>(pos_data.size()),
		norm_data.data(), static_cast<int>(norm_data.size()),
		coord_data.data(), static_cast<int>(coord_data.size()),
		idx_data.data(), static_cast<int>(idx_data.size()), out_mesh.get());
}

void cMeshUtil::BuildCapsuleMesh(int stacks, int slices, std::unique_ptr<cDrawMesh>& out_mesh)
{
	const float r = 1;
	const float h = 1;

	stacks = std::max(stacks / 2, 1) * 2;
	int num_triangles = 2 * slices * (stacks - 1) + 2 * slices;

	std::vector<float> pos_data;
	std::vector<float> norm_data;
	std::vector<float> coord_data;
	std::vector<int> idx_data;
	pos_data.reserve(num_triangles * gVertsPerTriangle * gPosDim);
	norm_data.reserve(num_triangles * gVertsPerTriangle * gNormDim);
	coord_data.reserve(num_triangles * gVertsPerTriangle * gCoordDim);
	idx_data.resize(num_triangles * gVertsPerTriangle);

	for (int i = 0; i < stacks; ++i)
	{
		float rho0 = 0.5 * M_PI - (i * M_PI) / stacks;
		float rho1 = 0.5 * M_PI - ((i + 1) * M_PI) / stacks;
		float coord_v0 = 1 - static_cast<float>(i) / stacks;
		float coord_v1 = 1 - static_cast<float>(i + 1) / stacks;

		float y0 = r * std::sin(rho0);
		float r0 = r * std::cos(rho0);
		float y1 = r * std::sin(rho1);
		float r1 = r * std::cos(rho1);
		float y2 = y1;
		float y3 = y0;

		float h_offset = -0.5 * ((i < stacks / 2) ? -h : h);
		for (int j = 0; j < slices; ++j)
		{
			float theta0 = (j * 2 * M_PI) / slices;
			float theta1 = ((j + 1) * 2 * M_PI) / slices;
			float coord_u0 = static_cast<float>(j) / slices;
			float coord_u1 = static_cast<float>(j + 1) / slices;

			float x0 = r0 * std::cos(theta0);
			float z0 = r0 * std::sin(-theta0);
			float u0 = coord_u0;
			float v0 = coord_v0;
			float x1 = r1 * std::cos(theta0);
			float z1 = r1 * std::sin(-theta0);
			float u1 = coord_u0;
			float v1 = coord_v1;
			float x2 = r1 * std::cos(theta1);
			float z2 = r1 * std::sin(-theta1);
			float u2 = coord_u1;
			float v2 = coord_v1;
			float x3 = r0 * std::cos(theta1);
			float z3 = r0 * std::sin(-theta1);
			float u3 = coord_u1;
			float v3 = coord_v0;

			if (i != stacks - 1)
			{
				pos_data.push_back(x0);
				pos_data.push_back(y0 + h_offset);
				pos_data.push_back(z0);
				norm_data.push_back(x0);
				norm_data.push_back(y0);
				norm_data.push_back(z0);
				coord_data.push_back(u0);
				coord_data.push_back(v0);

				pos_data.push_back(x1);
				pos_data.push_back(y1 + h_offset);
				pos_data.push_back(z1);
				norm_data.push_back(x1);
				norm_data.push_back(y1);
				norm_data.push_back(z1);
				coord_data.push_back(u1);
				coord_data.push_back(v1);

				pos_data.push_back(x2);
				pos_data.push_back(y2 + h_offset);
				pos_data.push_back(z2);
				norm_data.push_back(x2);
				norm_data.push_back(y2);
				norm_data.push_back(z2);
				coord_data.push_back(u2);
				coord_data.push_back(v2);
			}

			if (i != 0)
			{
				pos_data.push_back(x2);
				pos_data.push_back(y2 + h_offset);
				pos_data.push_back(z2);
				norm_data.push_back(x2);
				norm_data.push_back(y2);
				norm_data.push_back(z2);
				coord_data.push_back(u2);
				coord_data.push_back(v2);

				pos_data.push_back(x3);
				pos_data.push_back(y3 + h_offset);
				pos_data.push_back(z3);
				norm_data.push_back(x3);
				norm_data.push_back(y3);
				norm_data.push_back(z3);
				coord_data.push_back(u3);
				coord_data.push_back(v3);

				pos_data.push_back(x0);
				pos_data.push_back(y0 + h_offset);
				pos_data.push_back(z0);
				norm_data.push_back(x0);
				norm_data.push_back(y0);
				norm_data.push_back(z0);
				coord_data.push_back(u0);
				coord_data.push_back(v0);
			}
		}
	}

	for (int i = 0; i < slices; ++i)
	{
		double theta0 = (i * 2 * M_PI) / slices;
		double theta1 = ((i + 1) * 2 * M_PI) / slices;

		double x0 = r * std::cos(theta0);
		double z0 = r * std::sin(-theta0);
		double u0 = static_cast<float>(i) / slices;

		double x1 = r * std::cos(theta1);
		double z1 = r * std::sin(-theta1);
		double u1 = static_cast<float>(i + 1) / slices;

		tVector n0 = tVector(x0, 0, z0, 0).normalized();
		tVector n1 = tVector(x1, 0, z1, 0).normalized();

		pos_data.push_back(x0);
		pos_data.push_back(-0.5 * h);
		pos_data.push_back(z0);
		pos_data.push_back(x1);
		pos_data.push_back(-0.5 * h);
		pos_data.push_back(z1);
		pos_data.push_back(x1);
		pos_data.push_back(0.5 * h);
		pos_data.push_back(z1);
		pos_data.push_back(x1);
		pos_data.push_back(0.5 * h);
		pos_data.push_back(z1);
		pos_data.push_back(x0);
		pos_data.push_back(0.5 * h);
		pos_data.push_back(z0);
		pos_data.push_back(x0);
		pos_data.push_back(-0.5 * h);
		pos_data.push_back(z0);

		norm_data.push_back(n0[0]);
		norm_data.push_back(n0[1]);
		norm_data.push_back(n0[2]);
		norm_data.push_back(n1[0]);
		norm_data.push_back(n1[1]);
		norm_data.push_back(n1[2]);
		norm_data.push_back(n1[0]);
		norm_data.push_back(n1[1]);
		norm_data.push_back(n1[2]);
		norm_data.push_back(n1[0]);
		norm_data.push_back(n1[1]);
		norm_data.push_back(n1[2]);
		norm_data.push_back(n0[0]);
		norm_data.push_back(n0[1]);
		norm_data.push_back(n0[2]);
		norm_data.push_back(n0[0]);
		norm_data.push_back(n0[1]);
		norm_data.push_back(n0[2]);

		coord_data.push_back(u0);
		coord_data.push_back(0.f);
		coord_data.push_back(u1);
		coord_data.push_back(0.f);
		coord_data.push_back(u1);
		coord_data.push_back(1);
		coord_data.push_back(u1);
		coord_data.push_back(1.f);
		coord_data.push_back(u0);
		coord_data.push_back(1.f);
		coord_data.push_back(u0);
		coord_data.push_back(0.f);
	}

	for (int i = 0; i < static_cast<int>(idx_data.size()); ++i)
	{
		idx_data[i] = i;
	}

	out_mesh = std::unique_ptr<cDrawMesh>(new cDrawMesh());
	BuildDrawMesh(pos_data.data(), static_cast<int>(pos_data.size()),
		norm_data.data(), static_cast<int>(norm_data.size()),
		coord_data.data(), static_cast<int>(coord_data.size()),
		idx_data.data(), static_cast<int>(idx_data.size()), out_mesh.get());
}

void cMeshUtil::BuildCapsuleWireSimpleMesh(int stacks, int slices, std::unique_ptr<cDrawMesh>& out_mesh)
{
	const float r = 0.1;
	const float h = 0.21;
	int slices0 = 4;
	stacks = std::max(stacks / 2, 1) * 2;
	int num_lines = slices0 * stacks + 2 * slices + slices0;

	std::vector<float> pos_data;
	std::vector<float> norm_data;
	std::vector<float> coord_data;
	std::vector<int> idx_data;
	pos_data.reserve(num_lines * gVertsPerLine * gPosDim);
	norm_data.reserve(num_lines * gVertsPerLine * gNormDim);
	coord_data.reserve(num_lines * gVertsPerLine * gCoordDim);
	idx_data.resize(num_lines * gVertsPerLine);

	for (int i = 0; i < stacks; ++i)
	{
		float rho0 = 0.5 * M_PI - (i * M_PI) / stacks;
		float rho1 = 0.5 * M_PI - ((i + 1) * M_PI) / stacks;
		float coord_v0 = 1 - static_cast<float>(i) / stacks;
		float coord_v1 = 1 - static_cast<float>(i + 1) / stacks;

		float y0 = r * std::sin(rho0);
		float r0 = r * std::cos(rho0);
		float y1 = r * std::sin(rho1);
		float r1 = r * std::cos(rho1);

		float h_offset = -0.5 * ((i < stacks / 2) ? -h : h);

		for (int j = 0; j < slices0; ++j)
		{
			float theta0 = (j * 2 * M_PI) / slices0;
			float theta1 = ((j + 1) * 2 * M_PI) / slices0;
			float coord_u0 = static_cast<float>(j) / slices0;

			float x0 = r0 * std::cos(theta0);
			float z0 = r0 * std::sin(-theta0);
			float u0 = coord_u0;
			float v0 = coord_v0;
			float x1 = r1 * std::cos(theta0);
			float z1 = r1 * std::sin(-theta0);
			float u1 = coord_u0;
			float v1 = coord_v1;

			pos_data.push_back(x0);
			pos_data.push_back(y0 + h_offset);
			pos_data.push_back(z0);
			norm_data.push_back(x0);
			norm_data.push_back(y0);
			norm_data.push_back(z0);
			coord_data.push_back(u0);
			coord_data.push_back(v0);

			pos_data.push_back(x1);
			pos_data.push_back(y1 + h_offset);
			pos_data.push_back(z1);
			norm_data.push_back(x1);
			norm_data.push_back(y1);
			norm_data.push_back(z1);
			coord_data.push_back(u1);
			coord_data.push_back(v1);
		}
	}

	for (int j = 0; j < slices; ++j)
	{
		float theta0 = (j * 2 * M_PI) / slices;
		float theta1 = ((j + 1) * 2 * M_PI) / slices;
		float coord_u0 = static_cast<float>(j) / slices;
		float coord_v0 = 0.5;
		float coord_v1 = 0.5;

		float x0 = r * std::cos(theta0);
		float y0 = 0;
		float z0 = r * std::sin(-theta0);
		float u0 = coord_u0;
		float v0 = coord_v0;
		float x1 = r * std::cos(theta1);
		float y1 = 0;
		float z1 = r * std::sin(-theta1);
		float u1 = coord_u0;
		float v1 = coord_v1;

		pos_data.push_back(x0);
		pos_data.push_back(y0 - 0.5 * h);
		pos_data.push_back(z0);
		pos_data.push_back(x1);
		pos_data.push_back(y1 - 0.5 * h);
		pos_data.push_back(z1);
		pos_data.push_back(x0);
		pos_data.push_back(y0 + 0.5 * h);
		pos_data.push_back(z0);
		pos_data.push_back(x1);
		pos_data.push_back(y1 + 0.5 * h);
		pos_data.push_back(z1);
		
		norm_data.push_back(x0);
		norm_data.push_back(y0);
		norm_data.push_back(z0);
		norm_data.push_back(x1);
		norm_data.push_back(y1);
		norm_data.push_back(z1);
		norm_data.push_back(x0);
		norm_data.push_back(y0);
		norm_data.push_back(z0);
		norm_data.push_back(x1);
		norm_data.push_back(y1);
		norm_data.push_back(z1);
		
		coord_data.push_back(u0);
		coord_data.push_back(v0);
		coord_data.push_back(u1);
		coord_data.push_back(v1);
		coord_data.push_back(u0);
		coord_data.push_back(v0);
		coord_data.push_back(u1);
		coord_data.push_back(v1);
	}

	for (int i = 0; i < slices0; ++i)
	{
		double theta0 = (i * 2 * M_PI) / slices0;
		double x0 = r * std::cos(theta0);
		double z0 = r * std::sin(-theta0);
		double u0 = static_cast<float>(i) / slices0;

		tVector n0 = tVector(x0, 0, z0, 0).normalized();

		pos_data.push_back(x0);
		pos_data.push_back(-0.5 * h);
		pos_data.push_back(z0);
		pos_data.push_back(x0);
		pos_data.push_back(0.5 * h);
		pos_data.push_back(z0);

		norm_data.push_back(n0[0]);
		norm_data.push_back(n0[1]);
		norm_data.push_back(n0[2]);
		norm_data.push_back(n0[0]);
		norm_data.push_back(n0[1]);
		norm_data.push_back(n0[2]);

		coord_data.push_back(u0);
		coord_data.push_back(0.f);
		coord_data.push_back(u0);
		coord_data.push_back(0.f);
	}

	for (int i = 0; i < static_cast<int>(idx_data.size()); ++i)
	{
		idx_data[i] = i;
	}

	out_mesh = std::unique_ptr<cDrawMesh>(new cDrawMesh());
	BuildDrawMesh(pos_data.data(), static_cast<int>(pos_data.size()),
		norm_data.data(), static_cast<int>(norm_data.size()),
		coord_data.data(), static_cast<int>(coord_data.size()),
		idx_data.data(), static_cast<int>(idx_data.size()), out_mesh.get());
}

bool cMeshUtil::LoadObj(const std::string& path, cDrawMesh& out_mesh)
{
	bool succ = false;

	int data_size = 0;
	int num_indices = 0;
	int num_attrib;
	unsigned char* data = nullptr;
	int* indices = nullptr;
	obj_attrib_info* attr_info = nullptr;

	cObjLoader obj_loader = cObjLoader(path);
	obj_loader.objExportGLSeparate(data_size, data, num_indices, indices, num_attrib, attr_info);

	assert(num_attrib >= 1);
	succ = num_attrib >= 1;

	int pos_size = 0;
	const float* pos_data = nullptr;
	const obj_attrib_info& pos_attrib = attr_info[0];
	assert(pos_attrib.attrib_size == gPosDim * sizeof(float));
	pos_size = static_cast<int>(obj_loader.vertices.size());
	pos_data = reinterpret_cast<const float*>(&data[pos_attrib.data_offset]);

	BuildDrawMesh(pos_data, pos_size, indices, num_indices, &out_mesh);

	return succ;
}


bool cMeshUtil::RayIntersectTriangle(const tVector& start, const tVector& end, const tVector& v0, const tVector& v1, const tVector& v2,
									tVector& out_pos)
{
	tVector d = end - start;
	tVector e1 = v1 - v0;
	tVector e2 = v2 - v0;
	tVector h = d.cross3(e2);
	double a = e1.dot(h);

	if (a > -0.00001 && a < 0.00001)
	{
		return false;
	}

	double f = 1 / a;
	tVector s = start - v0;
	double u = f * s.dot(h);

	if (u < 0.0 || u > 1.0)
	{
		return false;
	}

	tVector q = s.cross3(e1);
	double v = f * d.dot(q);
	if (v < 0.0 || u + v > 1.0)
	{
		return false;
	}

	double t = f * e2.dot(q);
	if (t > 0.00001)
	{
		out_pos = start + t * d;
		return true;
	}

	return false;
}