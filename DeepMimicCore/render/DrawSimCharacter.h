#pragma once

#include "sim/SimCharacter.h"
#include "sim/Ground.h"
#include "sim/DeepMimicCharController.h"
#include "util/CircularBuffer.h"
#include "render/Camera.h"

class cDrawSimCharacter
{
public:
	static void Draw(const cSimCharacter& character, const tVector& fill_tint, const tVector& line_col, bool enable_draw_shape = false);
	static void DrawCoM(const cSimCharacter& character, double marker_size, double vel_scale,
						const tVector& col, const tVector& offset);
	static void DrawTorque(const cSimCharacter& character, const tVector& offset);
	static void DrawBodyVel(const cSimCharacter& character, double lin_vel_scale, double ang_vel_scale, const tVector& offset);
	static void DrawInfoValLog(const cCircularBuffer<double>& val_log, const cCamera& cam);
	
protected:
	
	static void DrawSimBody(const cSimCharacter& character, const tVector& fill_tint, const tVector& line_col);
};
