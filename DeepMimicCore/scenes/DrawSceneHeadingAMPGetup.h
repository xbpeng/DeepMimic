#pragma once

#include "scenes/DrawSceneHeadingAMP.h"

class cDrawSceneHeadingAMPGetup : virtual public cDrawSceneHeadingAMP
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawSceneHeadingAMPGetup();
	virtual ~cDrawSceneHeadingAMPGetup();

protected:

	virtual void BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const;
};
