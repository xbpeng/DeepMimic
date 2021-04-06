#include "DrawKinTree.h"

#include <math.h>

#include "render/DrawUtil.h"

void cDrawKinTree::Draw(const Eigen::MatrixXd& joint_desc, const Eigen::VectorXd& pose, double link_width, const tVector& fill_col, const tVector& line_col)
{
	int root_id = cKinTree::GetRootID();
	cDrawUtil::SetLineWidth(1);
	DrawTree(joint_desc, pose, root_id, link_width, fill_col, line_col);
}

void cDrawKinTree::DrawTree(const Eigen::MatrixXd& joint_desc, const Eigen::VectorXd& pose, int joint_id, double link_width,
	const tVector& fill_col, const tVector& line_col)
{
	const double node_radius = link_width;

	if (joint_id != cKinTree::gInvalidJointID)
	{
		bool has_parent = cKinTree::HasParent(joint_desc, joint_id);
		if (has_parent)
		{
			tVector attach_pt = cKinTree::GetAttachPt(joint_desc, joint_id);
			double len = attach_pt.norm();

			tVector attach_dir = attach_pt.normalized();
			const tVector up = tVector(0, 1, 0, 0);
			tMatrix rot_mat = cMathUtil::DirToRotMat(attach_dir, up);

			tVector pos = (len / 2) * attach_dir;
			tVector size = tVector(link_width, len, link_width, 0);

			// draw link
			cDrawUtil::PushMatrixView();
			cDrawUtil::Translate(pos);
			cDrawUtil::MultMatrixView(rot_mat);
			cDrawUtil::Rotate(0.5 * M_PI, tVector(1, 0, 0, 0));

			cDrawUtil::SetColor(tVector(fill_col[0], fill_col[1], fill_col[2], fill_col[3]));
			cDrawUtil::DrawBox(tVector::Zero(), size, cDrawUtil::eDrawSolid);

			if (line_col[3] > 0)
			{
				cDrawUtil::SetColor(tVector(line_col[0], line_col[1], line_col[2], line_col[3]));
				cDrawUtil::DrawBox(tVector::Zero(), size, cDrawUtil::eDrawWireSimple);
			}

			cDrawUtil::PopMatrixView();
		}

		cDrawUtil::PushMatrixView();
		tMatrix m = cKinTree::ChildParentTrans(joint_desc, pose, joint_id);
		cDrawUtil::MultMatrixView(m);

		// draw node
		cDrawUtil::SetColor(tVector(fill_col[0] * 0.25, fill_col[1] * 0.25, fill_col[2] * 0.25, fill_col[3]));
		cDrawUtil::DrawSphere(node_radius);

		Eigen::VectorXi children;
		cKinTree::FindChildren(joint_desc, joint_id, children);
		for (int i = 0; i < children.size(); ++i)
		{
			int child_id = children[i];
			DrawTree(joint_desc, pose, child_id, link_width, fill_col, line_col);
		}

		cDrawUtil::PopMatrixView();
	}
}