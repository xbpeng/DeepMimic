#pragma once

#include "sim/Ground.h"
#include "render/DrawMesh.h"

class cDrawGround
{
public:
	static void BuildMesh(const cGround* ground, cDrawMesh* out_mesh);

protected:
	static void BuildMeshPlane(const cGround* ground, cDrawMesh* out_mesh);
};