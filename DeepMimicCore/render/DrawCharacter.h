#pragma once

#include "anim/Character.h"
#include "render/DrawMesh.h"

class cDrawCharacter
{
public:
	static void Draw(const cCharacter& character, double link_width, const tVector& fill_col, const tVector& line_col);
	static void DrawPose(const cCharacter& character, const Eigen::VectorXd& pose, const tVector& fill_tint, const tVector& line_col);
	static void DrawHeading(const cCharacter& character, double arrow_size, const tVector& arrow_col, const tVector& offset);

protected:
	static void DrawShape(const cKinTree::tDrawShapeDef& def, const tMatrix& parent_world_trans,
							const tVector& fill_tint, const tVector& line_col);
	static void DrawShapeMesh(const cKinTree::tDrawShapeDef& def, const tMatrix& parent_world_trans, 
							cDrawMesh& mesh, const tVector& fill_tint);

	static void DrawCharShapes(const cCharacter& character, const tVector& fill_tint, const tVector& line_col);
	static void DrawPoseCharShapes(const cCharacter& character, const Eigen::VectorXd& pose, const tVector& fill_tint, const tVector& line_col);
	
	static void DrawShapeBox(const cKinTree::tDrawShapeDef& def, const tMatrix& parent_world_trans, 
							const tVector& fill_tint, const tVector& line_col);
	static void DrawShapeCapsule(const cKinTree::tDrawShapeDef& def, const tMatrix& parent_world_trans, 
							const tVector& fill_tint, const tVector& line_col);
	static void DrawShapeSphere(const cKinTree::tDrawShapeDef& def, const tMatrix& parent_world_trans, 
							const tVector& fill_tint, const tVector& line_col);
	static void DrawShapeCylinder(const cKinTree::tDrawShapeDef& def, const tMatrix& parent_world_trans, 
							const tVector& fill_tint, const tVector& line_col);
};