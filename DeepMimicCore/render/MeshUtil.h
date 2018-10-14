#pragma once

#include <memory>
#include "render/DrawMesh.h"
#include "util/MathUtil.h"

class cMeshUtil
{
public:

	enum eAttribute
	{
		eAttributePosition,
		eAttributeNormal,
		eAttributeCoord,
		eAttributeMax
	};

	struct tVertex
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		tVector mPosition;
		tVector mNormal;
		Eigen::Vector2d mCoord;

		tVertex();
	};

	struct tRayTestResult
	{
		int mFace;
		double mDist;
		tVector mIntersect;

		tRayTestResult();
	};

	typedef Eigen::Vector3i tFace;

	static const int gPosDim = 3;
	static const int gNormDim = 3;
	static const int gCoordDim = 2;
	static const int gVertsPerTriangle = 3;
	static const int gVertsPerLine = 2;

	static void BuildDrawMesh(const float* pos_data, int pos_size, const int* idx_data, int idx_size,
							cDrawMesh* out_mesh);
	static void BuildDrawMesh(const float* pos_data, int pos_size, const float* norm_data, int norm_size, 
							const float* coord_data, int coord_size, const int* idx_data, int idx_size,
							cDrawMesh* out_mesh);

	static tVertex GetVertex(int v, const cDrawMesh& mesh);
	static tFace GetFace(int f, const cDrawMesh& mesh);

	static void ExpandFaces(const cDrawMesh& mesh, std::shared_ptr<cDrawMesh>& out_mesh);
	static void RayTest(const tVector& start, const tVector& end, const cDrawMesh& mesh, std::vector<tRayTestResult>& out_result);

	static void BuildPointMesh(std::unique_ptr<cDrawMesh>& out_mesh);
	static void BuildLineMesh(std::unique_ptr<cDrawMesh>& out_mesh);
	static void BuildQuadMesh(std::unique_ptr<cDrawMesh>& out_mesh);
	static void BuildBoxSolidMesh(std::unique_ptr<cDrawMesh>& out_mesh);
	static void BuildBoxWireMesh(std::unique_ptr<cDrawMesh>& out_mesh);
	static void BuildSphereMesh(int stacks, int slices, std::unique_ptr<cDrawMesh>& out_mesh);
	static void BuildSphereWireSimpleMesh(int stacks, int slices, std::unique_ptr<cDrawMesh>& out_mesh);
	static void BuildDiskMesh(int slices, std::unique_ptr<cDrawMesh>& out_mesh);
	static void BuildTriangleMesh(std::unique_ptr<cDrawMesh>& out_mesh);
	static void BuildCylinder(int slices, std::unique_ptr<cDrawMesh>& out_mesh);
	static void BuildCylinderWireSimple(int slices, std::unique_ptr<cDrawMesh>& out_mesh);
	static void BuildCone(int slices, std::unique_ptr<cDrawMesh>& out_mesh);
	static void BuildConeWireSimple(int slices, std::unique_ptr<cDrawMesh>& out_mesh);
	static void BuildCapsuleMesh(int stacks, int slices, std::unique_ptr<cDrawMesh>& out_mesh);
	static void BuildCapsuleWireSimpleMesh(int stacks, int slices, std::unique_ptr<cDrawMesh>& out_mesh);

	static bool LoadObj(const std::string& path, cDrawMesh& out_mesh);

protected:

	static bool RayIntersectTriangle(const tVector& start, const tVector& end, const tVector& v0, const tVector& v1, const tVector& v2,
									tVector& out_pos);
};