#pragma once

#include "scenes/DrawSceneImitate.h"

class cDrawSceneImitateAMP : virtual public cDrawSceneImitate
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cDrawSceneImitateAMP();
	virtual ~cDrawSceneImitateAMP();

protected:

	virtual void BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const;
};
