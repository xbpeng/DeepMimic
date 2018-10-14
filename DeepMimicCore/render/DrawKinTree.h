#pragma once

#include "anim/KinTree.h"
#include <set>

class cDrawKinTree
{
public:
	static void Draw(const Eigen::MatrixXd& joint_desc, const Eigen::VectorXd& pose, double link_width, const tVector& fill_col, const tVector& line_col);
	static void DrawTree(const Eigen::MatrixXd& joint_desc, const Eigen::VectorXd& pose, int joint_id, double link_width, const tVector& fill_col, const tVector& line_col);
};
