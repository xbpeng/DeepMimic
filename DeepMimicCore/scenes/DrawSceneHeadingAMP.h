#pragma once

#include "scenes/DrawSceneTargetAMP.h"

class cDrawSceneHeadingAMP : virtual public cDrawSceneTargetAMP
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawSceneHeadingAMP();
	virtual ~cDrawSceneHeadingAMP();

	virtual void Reset();
	virtual void Keyboard(unsigned char key, double device_x, double device_y);

protected:

	virtual void BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const;

	virtual void KeyboardHeading(unsigned char key, double device_x, double device_y);

	virtual double GetTargetHeading() const;
	virtual bool EnableTargetPos() const;
	virtual void SetTargetPos(const tVector& pos);
	virtual double GetTargetSpeed() const;
	virtual void SetTargetSpeed(double speed);
	virtual void ChangeTargetSpeed(double delta);

	virtual void DrawMisc() const;
	virtual void DrawTargetPos(const tVector& target_pos) const;
	virtual void DrawTargetHeading(double heading) const;
};
