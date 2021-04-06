#include "DrawSimCharacter.h"
#include "DrawCharacter.h"
#include "sim/SimBox.h"
#include "render/DrawObj.h"
#include "render/DrawPerturb.h"
#include "render/GraphUtil.h"

void cDrawSimCharacter::Draw(const cSimCharacter& character, const tVector& fill_tint, const tVector& line_col, bool enable_draw_shape)
{
	bool has_draw_shapes = character.HasDrawShapes();
	if (has_draw_shapes && enable_draw_shape)
	{
		cDrawCharacter::Draw(character, 0, fill_tint, line_col);
	}
	else
	{
		DrawSimBody(character, fill_tint, line_col);
	}
}

void cDrawSimCharacter::DrawCoM(const cSimCharacter& character, double marker_size, double vel_scale,
	const tVector& col, const tVector& offset)
{
	const double arrow_size = marker_size * 0.65;
	tVector com = character.CalcCOM();
	tVector com_vel = character.CalcCOMVel();

	cDrawUtil::SetLineWidth(4);
	cDrawUtil::SetColor(tVector(col[0], col[1], col[2], col[3]));
	cDrawUtil::DrawCross(com + offset, marker_size);
	cDrawUtil::DrawArrow2D(com + offset, com + offset + com_vel * vel_scale, arrow_size);
}

void cDrawSimCharacter::DrawTorque(const cSimCharacter& character, const tVector& offset)
{
	int num_joints = character.GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		const cSimBodyJoint& joint = character.GetJoint(j);
		if (joint.IsValid())
		{
			tVector torque = joint.GetTotalTorque();
			tVector pos = joint.CalcWorldPos();
			cDrawPerturb::DrawTorque(pos + offset, torque);
		}
	}
}

void cDrawSimCharacter::DrawBodyVel(const cSimCharacter& character, double lin_vel_scale, double ang_vel_scale, const tVector& offset)
{
	const double lin_width = 3;
	const tVector lin_col = tVector(0, 0, 1, 0.5);
	const tVector ang_col = tVector(0, 0.75, 0, 0.5);

	cDrawUtil::SetLineWidth(lin_width);
	for (int i = 0; i < character.GetNumBodyParts(); ++i)
	{
		if (character.IsValidBodyPart(i))
		{
			const auto& body = character.GetBodyPart(i);
			tVector pos = body->GetPos();
			tVector vel = body->GetLinearVelocity();
			tVector ang_vel = body->GetAngularVelocity();
			pos += offset;

			cDrawUtil::SetColor(lin_col);
			cDrawUtil::DrawLine(pos, pos + lin_vel_scale * vel);

			cDrawUtil::SetColor(ang_col);
			cDrawUtil::DrawLine(pos, pos + ang_vel_scale * ang_vel);
		}
	}
}

void cDrawSimCharacter::DrawInfoValLog(const cCircularBuffer<double>& val_log, const cCamera& cam)
{
	const double min_val = 0;
	const double max_val = 1;

	int num_val = static_cast<int>(val_log.GetSize());
	double aspect = cam.GetAspectRatio();

	const double h = 0.4;
	const double w = 16.0 / 9 * h / aspect;

	tVector origin = tVector::Zero();
	origin[0] = 1 - w * 1.05;
	origin[1] = 1 - h * 1.05;
	origin[2] = -1;

	int capacity = static_cast<int>(val_log.GetCapacity());

	cDrawUtil::SetLineWidth(1);
	cDrawUtil::SetColor(tVector(1, 1, 1, 0.5));
	cDrawUtil::DrawRect(origin + 0.5 * tVector(w, h, 0, 0), tVector(w, h, 0, 0));
	cDrawUtil::SetColor(tVector(0, 0, 0, 1));
	cDrawUtil::DrawRect(origin + 0.5 * tVector(w, h, 0, 0), tVector(w, h, 0, 0), cDrawUtil::eDrawWireSimple);

	cDrawUtil::SetLineWidth(1);
	cDrawUtil::SetPointSize(2);
	cDrawUtil::SetColor(tVector(1, 0, 0, 0.5));

	if (num_val > 0)
	{
		double prev_val = val_log[0];
		for (int i = 1; i < num_val; ++i)
		{
			double curr_val = val_log[i];

			tVector a = tVector::Zero();
			tVector b = tVector::Zero();

			a[0] = w * (i - 1.0) / (capacity - 1.0);
			b[0] = w * (i) / (capacity - 1.0);

			a[1] = h * cMathUtil::Clamp((prev_val - min_val) / (max_val - min_val), 0.0, 1.0);
			b[1] = h * cMathUtil::Clamp((curr_val - min_val) / (max_val - min_val), 0.0, 1.0);

			a += origin;
			b += origin;

			cDrawUtil::DrawLine(a, b);
			cDrawUtil::DrawPoint(b);
			prev_val = curr_val;
		}
	}
}

void cDrawSimCharacter::DrawSimBody(const cSimCharacter& character, const tVector& fill_tint, const tVector& line_col)
{
	const tVector gContactCol = tVector(0.5, 0.75, 0.5, 1);

	cDrawUtil::SetLineWidth(1);
	for (int i = 0; i < character.GetNumBodyParts(); ++i)
	{
		if (character.IsValidBodyPart(i))
		{
			tVector col;
			const auto& curr_part = character.GetBodyPart(i);

			if (curr_part->IsInContact())
			{
				col = gContactCol;
			}
			else
			{
				col = character.GetPartColor(i);
				col = col.cwiseProduct(fill_tint);
			}

			cDrawUtil::SetColor(col);
			cDrawObj::Draw(curr_part.get(), cDrawUtil::eDrawSolid);

			if (line_col[3] > 0)
			{
				cDrawUtil::SetColor(line_col);
				cDrawObj::Draw(curr_part.get(), cDrawUtil::eDrawWireSimple);
			}
		}
	}
}