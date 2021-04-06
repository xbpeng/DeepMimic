#pragma once

#include "DrawSceneTargetAMP.h"

class cDrawSceneStrikeAMP : virtual public cDrawSceneTargetAMP
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawSceneStrikeAMP();
	virtual ~cDrawSceneStrikeAMP();

protected:

	virtual void BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const;

	virtual void AddTraces();

	virtual void DrawTargetPos(const tVector& target_pos) const;
	virtual void SetTargetPos(const tVector& pos);
};