#pragma once

#include "scenes/DrawSceneImitateAMP.h"

class cDrawSceneTargetAMP : virtual public cDrawSceneImitateAMP
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawSceneTargetAMP();
	virtual ~cDrawSceneTargetAMP();

	virtual void Reset();

protected:

	virtual void BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const;

	virtual tVector GetTargetPos() const;
	virtual void SetTargetPos(const tVector& pos);
	virtual bool CheckTargetSucc() const;

	virtual void HandleRayTest(const cWorld::tRayTestResult& result);

	virtual void DrawMisc() const;
	virtual void DrawTargetPos(const tVector& target_pos) const;
};
